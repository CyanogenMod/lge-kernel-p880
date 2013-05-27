#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include <asm/cputime.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "../../arch/arm/mach-tegra/tegra_pmqos.h"

/******************** Tunable parameters: ********************/

/*
 * The "ideal" frequency to use. The governor will ramp up faster
 * towards the ideal frequency and slower after it has passed it. Similarly,
 * lowering the frequency towards the ideal frequency is faster than below it.
 */

#define DEFAULT_SUSPEND_IDEAL_FREQ 340000
static unsigned int suspend_ideal_freq;

#define DEFAULT_AWAKE_IDEAL_FREQ 475000
static unsigned int awake_ideal_freq;

/*
 * Freqeuncy delta when ramping up above the ideal freqeuncy.
 * Zero disables and causes to always jump straight to max frequency.
 * When below the ideal freqeuncy we always ramp up to the ideal freq.
 */
#define DEFAULT_RAMP_UP_STEP 300000
static unsigned int ramp_up_step;

/*
 * Freqeuncy delta when ramping down below the ideal freqeuncy.
 * Zero disables and will calculate ramp down according to load heuristic.
 * When above the ideal freqeuncy we always ramp down to the ideal freq.
 */
#define DEFAULT_RAMP_DOWN_STEP 200000
static unsigned int ramp_down_step;

/*
 * CPU freq will be increased if measured load > max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 75
static unsigned int max_cpu_load;

/*
 * CPU freq will be decreased if measured load < min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 40
static unsigned int min_cpu_load;

/*
 * The minimum amount of time in nsecs to spend at a frequency before we can ramp up.
 * Notice we ignore this when we are below the ideal frequency.
 */
#define DEFAULT_UP_RATE 20000
static unsigned int up_rate;

/*
 * The minimum amount of time in nsecs to spend at a frequency before we can ramp down.
 * Notice we ignore this when we are above the ideal frequency.
 */
#define DEFAULT_DOWN_RATE 60000
static unsigned int down_rate;

/* in nsecs */
#define DEFAULT_SAMPLING_RATE 20000
static unsigned int sampling_rate;

/* in nsecs */
#define DEFAULT_INPUT_BOOST_DURATION 50000000
static unsigned int input_boost_duration;

static unsigned int touch_poke_freq = 640000;
static bool touch_poke = true;

/*
 * should ramp_up steps during boost be possible
 */
static bool ramp_up_during_boost = true;

/*
 * external boost interface - boost if duration is written
 * to sysfs for boost_duration
 */
static unsigned int boost_freq = 760000;
static bool boost = true;

/* in nsecs */
static unsigned int boost_duration = 0;

/* Consider IO as busy */
#define DEFAULT_IO_IS_BUSY 1
static unsigned int io_is_busy;

#define DEFAULT_IGNORE_NICE 1
static unsigned int ignore_nice;

#define GOVERNOR_NAME "pmc"
#define STRUCT_NAME cpufreq_gov_pmc
#define FUNC_NAME cpufreq_governor_pmc
#define CPUFR_NAME "cpufreq_pmc"

/*************** End of tunables ***************/

static int FUNC_NAME(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_PMC
static
#endif
struct cpufreq_governor STRUCT_NAME = { .name = GOVERNOR_NAME, .governor =
		FUNC_NAME, .max_transition_latency = 9000000, .owner =
		THIS_MODULE , };

#include "cpufreq_smartmax.c"

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_PMC
fs_initcall(cpufreq_smartmax_init);
#else
module_init(cpufreq_smartmax_init);
#endif

static void __exit cpufreq_smartmax_exit(void) {
	cpufreq_unregister_governor(&STRUCT_NAME);
}

module_exit(cpufreq_smartmax_exit);

MODULE_AUTHOR("maxwen");
MODULE_DESCRIPTION("'cpufreq_smartmax' - A smart cpufreq governor");
MODULE_LICENSE("GPL");
