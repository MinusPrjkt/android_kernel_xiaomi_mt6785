// SPDX-License-Identifier: GPL-2.0
/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/irq_work.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/sched/cpufreq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tick.h>
#include <trace/events/power.h>
#include <uapi/linux/sched/types.h>

/* Upper bound on how many entries efficient_freq[]/up_delay[] may hold. */
#define SCHEDHORIZON_MAX_STEPS	32

static unsigned int default_efficient_freq[] = {UINT_MAX};
static u64 default_up_delay[] = {0};

struct sugov_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		*efficient_freq;
	int			nefficient_freq;
	u64			*up_delay;
	int			nup_delay;

	/*
	 * Protects efficient_freq/up_delay and their counts against
	 * concurrent sysfs stores and governor reads.  raw_spinlock_t
	 * because do_freq_limit() runs from the scheduler's update_util
	 * callback in hardirq context.
	 */
	raw_spinlock_t		change_lock;
};

struct sugov_policy {
	struct cpufreq_policy	*policy;

	struct sugov_tunables	*tunables;
	struct list_head	tunables_hook;

	raw_spinlock_t		update_lock;	/* For shared policies */
	u64			last_freq_update_time;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	u64			first_hp_request_time;

	/*
	 * Per-policy stepping state.  Protected by tunables->change_lock;
	 * kept out of sugov_tunables because tunables may be shared across
	 * policies when !have_governor_per_policy().
	 */
	int			current_step;

	/* The next fields are only needed if fast switch cannot be used: */
	struct			irq_work irq_work;
	struct			kthread_work work;
	struct			mutex work_lock;
	struct			kthread_worker worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
};

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	u64			last_update;

	unsigned long		bw_dl;
	unsigned long		max;

	/* The field below is for single-CPU policies only: */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);

static inline int match_nearest_efficient_step(unsigned int freq, int maxstep,
					       unsigned int *freq_table)
{
	int i;

	for (i = 0; i < maxstep; i++) {
		if (freq_table[i] >= freq)
			break;
	}

	/* Callers must treat maxstep as "no match"; clamp to a safe index. */
	if (i == maxstep)
		i = maxstep - 1;

	return i;
}

static inline void do_freq_limit(struct sugov_policy *sg_policy,
				 unsigned int *freq, u64 time)
{
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned long flags;
	int step, neff, ndelay;

	/*
	 * do_freq_limit() runs from cpufreq_update_util() with IRQs
	 * disabled, so the lock must always be taken with irqsave.
	 */
	raw_spin_lock_irqsave(&tunables->change_lock, flags);

	step = sg_policy->current_step;
	neff = tunables->nefficient_freq;
	ndelay = tunables->nup_delay;

	if (step < 0 || step >= neff || step >= ndelay) {
		/* Self-heal: index out of range for one of the arrays. */
		sg_policy->current_step = 0;
		goto out;
	}

	if (*freq > tunables->efficient_freq[step] &&
	    !sg_policy->first_hp_request_time) {
		/* First request: pin to the current step's efficient freq. */
		*freq = tunables->efficient_freq[step];
		sg_policy->first_hp_request_time = time;
		goto out;
	}

	if (*freq < tunables->efficient_freq[step]) {
		/* Already below current step: drop to the nearest lower one. */
		step = match_nearest_efficient_step(*freq, neff,
						    tunables->efficient_freq);
		if (step < neff) {
			sg_policy->current_step = step;
			sg_policy->first_hp_request_time = 0;
		}
		goto out;
	}

	if (sg_policy->first_hp_request_time &&
	    time < sg_policy->first_hp_request_time + tunables->up_delay[step]) {
		/* Still within the dwell time: hold at current step. */
		*freq = tunables->efficient_freq[step];
		goto out;
	}

	if (step + 1 < neff && step + 1 < ndelay) {
		/* Dwell time elapsed: unlock the next step. */
		sg_policy->current_step++;
		sg_policy->first_hp_request_time = time;
		if (*freq > tunables->efficient_freq[sg_policy->current_step])
			*freq = tunables->efficient_freq[sg_policy->current_step];
	}
	/* else: at the top step, freq is passed through unchanged. */

out:
	raw_spin_unlock_irqrestore(&tunables->change_lock, flags);
}

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-CPU data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-CPU
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * This is needed on the slow switching platforms too to prevent CPUs
	 * going offline from leaving stale IRQ work items behind.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(sg_policy->limits_changed)) {
		sg_policy->limits_changed = false;
		sg_policy->need_freq_update = true;
	}

	return true;
}

/*
 * Check the up/down rate limit, picking the delay based on the direction
 * of the proposed frequency change relative to the last committed
 * frequency. This must be called after next_freq has been computed.
 */
static bool sugov_within_rate_limit(struct sugov_policy *sg_policy, u64 time,
				    unsigned int next_freq)
{
	s64 delta_ns;
	s64 delay_ns;

	if (unlikely(sg_policy->need_freq_update) || sg_policy->work_in_progress)
		return true;

	delta_ns = time - sg_policy->last_freq_update_time;

	delay_ns = (next_freq >= sg_policy->next_freq) ?
			sg_policy->up_rate_delay_ns :
			sg_policy->down_rate_delay_ns;

	return delta_ns >= delay_ns;
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (!sg_policy->need_freq_update) {
		if (sg_policy->next_freq == next_freq)
			return false;

		if (!sugov_within_rate_limit(sg_policy, time, next_freq))
			return false;
	} else {
		sg_policy->need_freq_update = false;
	}

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	return true;
}

static void sugov_fast_switch(struct sugov_policy *sg_policy, u64 time,
				  unsigned int next_freq)
{
	if (sugov_update_next_freq(sg_policy, time, next_freq))
		cpufreq_driver_fast_switch(sg_policy->policy, next_freq);
}

static void sugov_deferred_update(struct sugov_policy *sg_policy, u64 time,
				  unsigned int next_freq)
{
	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	if (!sg_policy->work_in_progress) {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: schedhorizon policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max, u64 time)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;
	unsigned int l_freq, h_freq;
	int idx;

	freq = map_util_freq(util, freq, max);

	do_freq_limit(sg_policy, &freq, time);

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->cached_raw_freq = freq;

	l_freq = cpufreq_driver_resolve_freq(policy, freq);
	if (!policy->freq_table)
		return l_freq;

	idx = cpufreq_frequency_table_target(policy, freq, CPUFREQ_RELATION_H);
	if (idx < 0)
		return l_freq;

	h_freq = policy->freq_table[idx].frequency;
	h_freq = clamp(h_freq, policy->min, policy->max);
	if (l_freq <= h_freq || l_freq == policy->min)
		return l_freq;

	/*
	 * Use the frequency step below if the calculated frequency is <20%
	 * higher than it.
	 */
	if (mult_frac(100, freq - h_freq, l_freq - h_freq) < 20)
		return h_freq;

	return l_freq;
}

/*
 * This function computes an effective utilization for the given CPU, to be
 * used for frequency selection given the linear relation: f = u * f_max.
 *
 * The scheduler tracks the following metrics:
 *
 *   cpu_util_{cfs,rt,dl,irq}()
 *   cpu_bw_dl()
 *
 * Where the cfs,rt and dl util numbers are tracked with the same metric and
 * synchronized windows and are thus directly comparable.
 *
 * The cfs,rt,dl utilization are the running times measured with rq->clock_task
 * which excludes things like IRQ and steal-time. These latter are then accrued
 * in the irq utilization.
 *
 * The DL bandwidth number otoh is not a measured metric but a value computed
 * based on the task model parameters and gives the minimal utilization
 * required to meet deadlines.
 */
unsigned long schedhorizon_cpu_util(int cpu, unsigned long util_cfs,
				 unsigned long max, enum schedutil_type type,
				 struct task_struct *p)
{
	unsigned long dl_util, util, irq;
	struct rq *rq = cpu_rq(cpu);

	if (!uclamp_is_used() &&
		type == FREQUENCY_UTIL && rt_rq_is_runnable(&rq->rt)) {
		return max;
	}

	/*
	 * Early check to see if IRQ/steal time saturates the CPU, can be
	 * because of inaccuracies in how we track these -- see
	 * update_irq_load_avg().
	 */
	irq = cpu_util_irq(rq);
	if (unlikely(irq >= max))
		return max;

	/*
	 * Because the time spent on RT/DL tasks is visible as 'lost' time to
	 * CFS tasks and we use the same metric to track the effective
	 * utilization (PELT windows are synchronized) we can directly add them
	 * to obtain the CPU's actual utilization.
	 *
	 * CFS and RT utilization can be boosted or capped, depending on
	 * utilization clamp constraints requested by currently RUNNABLE
	 * tasks.
	 * When there are no CFS RUNNABLE tasks, clamps are released and
	 * frequency will be gracefully reduced with the utilization decay.
	 */
	util = util_cfs + cpu_util_rt(rq);
	if (type == FREQUENCY_UTIL)
		util = uclamp_rq_util_with(rq, util, p);

	dl_util = cpu_util_dl(rq);

	/*
	 * For frequency selection we do not make cpu_util_dl() a permanent part
	 * of this sum because we want to use cpu_bw_dl() later on, but we need
	 * to check if the CFS+RT+DL sum is saturated (ie. no idle time) such
	 * that we select f_max when there is no idle time.
	 *
	 * NOTE: numerical errors or stop class might cause us to not quite hit
	 * saturation when we should -- something for later.
	 */
	if (util + dl_util >= max)
		return max;

	/*
	 * OTOH, for energy computation we need the estimated running time, so
	 * include util_dl and ignore dl_bw.
	 */
	if (type == ENERGY_UTIL)
		util += dl_util;

	/*
	 * There is still idle time; further improve the number by using the
	 * irq metric. Because IRQ/steal time is hidden from the task clock we
	 * need to scale the task numbers:
	 *
	 *              max - irq
	 *   U' = irq + --------- * U
	 *                 max
	 */
	util = scale_irq_capacity(util, irq, max);
	util += irq;

	/*
	 * Bandwidth required by DEADLINE must always be granted while, for
	 * FAIR and RT, we use blocked utilization of IDLE CPUs as a mechanism
	 * to gracefully reduce the frequency when no tasks show up for longer
	 * periods of time.
	 *
	 * Ideally we would like to set bw_dl as min/guaranteed freq and util +
	 * bw_dl as requested freq. However, cpufreq is not yet ready for such
	 * an interface. So, we only do the latter for now.
	 */
	if (type == FREQUENCY_UTIL)
		util += cpu_bw_dl(rq);

	return min(max, util);
}
EXPORT_SYMBOL_GPL(schedhorizon_cpu_util);

static unsigned long sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);
	unsigned long util = cpu_util_cfs(rq);
	unsigned long max = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

	sg_cpu->max = max;
	sg_cpu->bw_dl = cpu_bw_dl(rq);

	return schedhorizon_cpu_util(sg_cpu->cpu, util, max, FREQUENCY_UTIL, NULL);
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu, struct sugov_policy *sg_policy)
{
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
		sg_policy->limits_changed = true;
}

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;
	unsigned int cached_freq = sg_policy->cached_raw_freq;

	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	util = sugov_get_util(sg_cpu);
	max = sg_cpu->max;
	next_f = get_next_freq(sg_policy, util, max, time);
	/*
	 * Do not reduce the frequency if the CPU has not been idle
	 * recently, as the reduction is likely to be premature then.
	 */
	if (sugov_cpu_is_busy(sg_cpu) && next_f < sg_policy->next_freq) {
		next_f = sg_policy->next_freq;

		/* Restore cached freq as next_freq has changed */
		sg_policy->cached_raw_freq = cached_freq;
	}

	/*
	 * This code runs under rq->lock for the target CPU, so it won't run
	 * concurrently on two different CPUs for the same target and it is not
	 * necessary to acquire the lock in the fast switch case.
	 */
	if (sg_policy->policy->fast_switch_enabled) {
		sugov_fast_switch(sg_policy, time, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		sugov_deferred_update(sg_policy, time, next_f);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;

		j_util = sugov_get_util(j_sg_cpu);
		j_max = j_sg_cpu->max;

		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}
	}

	return get_next_freq(sg_policy, util, max, time);
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned int next_f;

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (sugov_should_update_freq(sg_policy, time)) {
		next_f = sugov_next_freq_shared(sg_cpu, time);

		if (sg_policy->policy->fast_switch_enabled)
			sugov_fast_switch(sg_policy, time, next_f);
		else
			sugov_deferred_update(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Hold sg_policy->update_lock shortly to handle the case where:
	 * in case sg_policy->next_freq is read here, and then updated by
	 * sugov_deferred_update() just before work_in_progress is set to false
	 * here, we may miss queueing the new update.
	 *
	 * Note: If a work was queued after the update_lock is released,
	 * sugov_work() will just be called again by kthread_work code; and the
	 * request will be proceed before the sugov thread sleeps.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	sg_policy->work_in_progress = false;
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

static unsigned int *resolve_data_freq(const char *buf, int *num_ret, size_t count)
{
	unsigned int *output;
	char *str, *cur, *token;
	unsigned int val;
	int i = 0;
	int ret;

	if (!count)
		return NULL;

	str = kmemdup(buf, count + 1, GFP_KERNEL);
	if (!str)
		return NULL;
	str[count] = '\0';

	/*
	 * Worst case: every byte is a separate one-character token, so
	 * count + 1 is a safe upper bound.  strsep() replaces delimiters
	 * with NULs, so it is used only once, on this private copy.
	 */
	output = kmalloc_array(count + 1, sizeof(*output), GFP_KERNEL);
	if (!output)
		goto err_free_str;

	cur = str;
	while ((token = strsep(&cur, " \t\n"))) {
		if (!*token)
			continue;
		if (i >= SCHEDHORIZON_MAX_STEPS)
			goto err_free_output;
		ret = kstrtouint(token, 10, &val);
		if (ret)
			goto err_free_output;
		output[i++] = val;
	}

	if (!i)
		goto err_free_output;

	kfree(str);
	*num_ret = i;
	return output;

err_free_output:
	kfree(output);
err_free_str:
	kfree(str);
	return NULL;
}

static u64 *resolve_data_delay(const char *buf, int *num_ret, size_t count)
{
	u64 *output;
	char *str, *cur, *token;
	u64 val;
	int i = 0;
	int ret;

	if (!count)
		return NULL;

	str = kmemdup(buf, count + 1, GFP_KERNEL);
	if (!str)
		return NULL;
	str[count] = '\0';

	output = kmalloc_array(count + 1, sizeof(*output), GFP_KERNEL);
	if (!output)
		goto err_free_str;

	cur = str;
	while ((token = strsep(&cur, " \t\n"))) {
		if (!*token)
			continue;
		if (i >= SCHEDHORIZON_MAX_STEPS)
			goto err_free_output;
		ret = kstrtou64(token, 10, &val);
		if (ret)
			goto err_free_output;
		if (val > U64_MAX / NSEC_PER_MSEC)
			goto err_free_output;
		output[i++] = val * NSEC_PER_MSEC;
	}

	if (!i)
		goto err_free_output;

	kfree(str);
	*num_ret = i;
	return output;

err_free_output:
	kfree(output);
err_free_str:
	kfree(str);
	return NULL;
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t
up_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;

	return count;
}

static ssize_t
down_rate_limit_us_store(struct gov_attr_set *attr_set, const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;

	return count;
}

static ssize_t efficient_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	unsigned int *freq;
	ssize_t ret = 0;
	int neff, i;

	raw_spin_lock_irqsave(&tunables->change_lock, flags);
	neff = tunables->nefficient_freq;
	freq = tunables->efficient_freq;
	for (i = 0; i < neff; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%u%c",
				  freq[i], (i + 1 < neff) ? ' ' : '\n');
	raw_spin_unlock_irqrestore(&tunables->change_lock, flags);

	return ret;
}

static ssize_t up_delay_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	u64 *delay;
	ssize_t ret = 0;
	int ndelay, i;

	raw_spin_lock_irqsave(&tunables->change_lock, flags);
	ndelay = tunables->nup_delay;
	delay = tunables->up_delay;
	for (i = 0; i < ndelay; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%llu%c",
				  delay[i] / NSEC_PER_MSEC,
				  (i + 1 < ndelay) ? ' ' : '\n');
	raw_spin_unlock_irqrestore(&tunables->change_lock, flags);

	return ret;
}

static ssize_t efficient_freq_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned long flags;
	unsigned int *new_efficient_freq, *old;
	int new_num, i;

	new_efficient_freq = resolve_data_freq(buf, &new_num, count);
	if (!new_efficient_freq)
		return -EINVAL;

	/*
	 * match_nearest_efficient_step() and do_freq_limit() require a
	 * strictly increasing, non-zero table; reject anything else
	 * instead of silently breaking the stepping state machine.
	 */
	for (i = 0; i < new_num; i++) {
		if (!new_efficient_freq[i] ||
		    (i && new_efficient_freq[i - 1] >= new_efficient_freq[i])) {
			kfree(new_efficient_freq);
			return -EINVAL;
		}
	}

	raw_spin_lock_irqsave(&tunables->change_lock, flags);
	old = tunables->efficient_freq;
	tunables->efficient_freq = new_efficient_freq;
	tunables->nefficient_freq = new_num;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->current_step = 0;
	raw_spin_unlock_irqrestore(&tunables->change_lock, flags);

	/*
	 * The old array can only be freed after dropping the lock: any
	 * reader that observed it is guaranteed to have finished using it.
	 */
	if (old != default_efficient_freq)
		kfree(old);

	return count;
}

static ssize_t up_delay_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned long flags;
	u64 *new_up_delay, *old;
	int new_num;

	new_up_delay = resolve_data_delay(buf, &new_num, count);
	if (!new_up_delay)
		return -EINVAL;

	raw_spin_lock_irqsave(&tunables->change_lock, flags);

	/*
	 * Delays are positional per step, so the array must match the
	 * number of efficient frequencies.  Write efficient_freq first,
	 * then up_delay with the same number of entries.
	 */
	if (new_num != tunables->nefficient_freq) {
		raw_spin_unlock_irqrestore(&tunables->change_lock, flags);
		kfree(new_up_delay);
		return -EINVAL;
	}

	old = tunables->up_delay;
	tunables->up_delay = new_up_delay;
	tunables->nup_delay = new_num;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->current_step = 0;
	raw_spin_unlock_irqrestore(&tunables->change_lock, flags);

	if (old != default_up_delay)
		kfree(old);

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr efficient_freq = __ATTR_RW(efficient_freq);
static struct governor_attr up_delay = __ATTR_RW(up_delay);

static struct attribute *sugov_attrs[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&efficient_freq.attr,
	&up_delay.attr,
	NULL
};
ATTRIBUTE_GROUPS(sugov);

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (tunables->efficient_freq != default_efficient_freq)
		kfree(tunables->efficient_freq);
	if (tunables->up_delay != default_up_delay)
		kfree(tunables->up_delay);
	kfree(tunables);
}

static struct kobj_type sugov_tunables_ktype = {
	.default_groups = sugov_groups,
	.sysfs_ops = &governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/

struct cpufreq_governor schedhorizon_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_attr attr = {
		.size		= sizeof(struct sched_attr),
		.sched_policy	= SCHED_DEADLINE,
		.sched_flags	= SCHED_FLAG_SUGOV,
		.sched_nice	= 0,
		.sched_priority	= 0,
		/*
		 * Fake (unused) bandwidth; workaround to "fix"
		 * priority inheritance.
		 */
		.sched_runtime	=  1000000,
		.sched_deadline = 10000000,
		.sched_period	= 10000000,
	};
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setattr_nocheck(thread, &attr);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_DEADLINE\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	if (!policy->dvfs_possible_from_any_cpu)
		kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		raw_spin_lock_init(&tunables->change_lock);
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	tunables->up_rate_limit_us = 500;
	tunables->down_rate_limit_us = 4000;
	tunables->efficient_freq = default_efficient_freq;
	tunables->nefficient_freq = ARRAY_SIZE(default_efficient_freq);
	tunables->up_delay = default_up_delay;
	tunables->nup_delay = ARRAY_SIZE(default_up_delay);

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   schedhorizon_gov.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		sugov_clear_global_tunables();

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns	= sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns	= sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	sg_policy->last_freq_update_time	= 0;
	sg_policy->next_freq			= 0;
	sg_policy->work_in_progress		= false;
	sg_policy->limits_changed		= false;
	sg_policy->cached_raw_freq		= 0;
	sg_policy->first_hp_request_time	= 0;
	sg_policy->current_step			= 0;

	sg_policy->need_freq_update = false;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu			= cpu;
		sg_cpu->sg_policy		= sg_policy;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
						 policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->limits_changed = true;
}

struct cpufreq_governor schedhorizon_gov = {
	.name			= "schedhorizon",
	.owner			= THIS_MODULE,
	.dynamic_switching	= true,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDHORIZON
struct cpufreq_governor *cpufreq_default_governor(void)
{
	return &schedhorizon_gov;
}
#endif

static int __init schedhorizon_register(void)
{
	return cpufreq_register_governor(&schedhorizon_gov);
}
fs_initcall(schedhorizon_register);
