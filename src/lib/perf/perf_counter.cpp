/****************************************************************************
 *
 * Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT of THE USE of THIS SOFTWARE, EVEN IF ADVISED of THE
 * POSSIBILITY of SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file perf_counter.cpp
 *
 * @brief Performance measuring tools.
 */

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <math.h>
#include <pthread.h>
#include <systemlib/err.h>

#include <px4_platform_common/atomic.h>

#include "perf_counter.h"

/**
 * Header common to all counters.
 */
struct perf_ctr_header {
	sq_entry_t		link;	/**< list linkage */
	enum perf_counter_type	type;	/**< counter type */
	const char		*name;	/**< counter name */
};

/**
 * PC_EVENT counter.
 */
struct perf_ctr_count : public perf_ctr_header {
	px4::atomic<uint64_t>	event_count{0};
};

/**
 * PC_ELAPSED counter.
 */
struct perf_ctr_elapsed : public perf_ctr_header {
	px4::atomic<uint64_t>	event_count{0};
	px4::atomic<uint64_t>	time_start{0};
	px4::atomic<uint64_t>	time_total{0};
	px4::atomic<uint32_t>	time_least{0};
	px4::atomic<uint32_t>	time_most{0};
	// Workaround for broken px4::atomic<float>: store float bits in atomic uint32_t
	px4::atomic<uint32_t>	mean_bits{0};
	px4::atomic<uint32_t>	M2_bits{0};
};

/**
 * PC_INTERVAL counter.
 */
struct perf_ctr_interval : public perf_ctr_header {
	px4::atomic<uint64_t>	event_count{0};
	px4::atomic<uint64_t>	time_event{0};
	px4::atomic<uint64_t>	time_first{0};
	px4::atomic<uint64_t>	time_last{0};
	px4::atomic<uint32_t>	time_least{0};
	px4::atomic<uint32_t>	time_most{0};
	// Workaround for broken px4::atomic<float>: store float bits in atomic uint32_t
	px4::atomic<uint32_t>	mean_bits{0};
	px4::atomic<uint32_t>	M2_bits{0};
};

/**
 * List of all known counters.
 */
static sq_queue_t	perf_counters = { nullptr, nullptr };

/**
 * mutex protecting access to the perf_counters linked list (which is read from & written to by different threads)
 */
pthread_mutex_t perf_counters_mutex = PTHREAD_MUTEX_INITIALIZER;


perf_counter_t
perf_alloc(enum perf_counter_type type, const char *name)
{
	perf_counter_t ctr = nullptr;

	switch (type) {
	case PC_COUNT:
		ctr = new perf_ctr_count();
		break;

	case PC_ELAPSED:
		ctr = new perf_ctr_elapsed();
		break;

	case PC_INTERVAL:
		ctr = new perf_ctr_interval();
		break;

	default:
		break;
	}

	if (ctr != nullptr) {
		ctr->type = type;
		ctr->name = name;
		pthread_mutex_lock(&perf_counters_mutex);
		sq_addfirst(&ctr->link, &perf_counters);
		pthread_mutex_unlock(&perf_counters_mutex);
	}

	return ctr;
}

perf_counter_t
perf_alloc_once(enum perf_counter_type type, const char *name)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		if (!strcmp(handle->name, name)) {
			if (type == handle->type) {
				/* they are the same counter */
				pthread_mutex_unlock(&perf_counters_mutex);
				return handle;

			} else {
				/* same name but different type, assuming this is an error and not intended */
				pthread_mutex_unlock(&perf_counters_mutex);
				return nullptr;
			}
		}

		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);

	/* if the execution reaches here, no existing counter of that name was found */
	return perf_alloc(type, name);
}

void
perf_free(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	pthread_mutex_lock(&perf_counters_mutex);
	sq_rem(&handle->link, &perf_counters);
	pthread_mutex_unlock(&perf_counters_mutex);

	switch (handle->type) {
	case PC_COUNT:
		delete (struct perf_ctr_count *)handle;
		break;

	case PC_ELAPSED:
		delete (struct perf_ctr_elapsed *)handle;
		break;

	case PC_INTERVAL:
		delete (struct perf_ctr_interval *)handle;
		break;

	default:
		break;
	}
}

void
perf_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count.fetch_add(1);
		break;

	case PC_INTERVAL:
		perf_count_interval(handle, hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_begin(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED:
		((struct perf_ctr_elapsed *)handle)->time_start.store(hrt_absolute_time());
		break;

	default:
		break;
	}
}

void
perf_end(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint64_t start_time = pce->time_start.load();

			if (start_time != 0) {
				perf_set_elapsed(handle, hrt_absolute_time() - start_time);
			}
		}
		break;

	default:
		break;
	}
}

void
perf_set_elapsed(perf_counter_t handle, int64_t elapsed)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;

			if (elapsed >= 0) {
				pce->time_total.fetch_add(elapsed);

				// Atomically update min time
				uint32_t prev_least = pce->time_least.load();
				while (((uint32_t)elapsed < prev_least || prev_least == 0) &&
				       !pce->time_least.compare_exchange(&prev_least, (uint32_t)elapsed));

				// Atomically update max time
				uint32_t prev_most = pce->time_most.load();
				while ((uint32_t)elapsed > prev_most &&
				       !pce->time_most.compare_exchange(&prev_most, (uint32_t)elapsed));

				// Welford's online algorithm for mean and variance, updated atomically.
				uint64_t n = pce->event_count.fetch_add(1) + 1;
				float dt = elapsed / 1e6f;

				// Atomically update mean using bit-casting in a CAS loop
				uint32_t old_mean_bits;
				uint32_t new_mean_bits;
				float old_mean;
				do {
					old_mean_bits = pce->mean_bits.load();
					memcpy(&old_mean, &old_mean_bits, sizeof(old_mean));
					const float new_mean = old_mean + (dt - old_mean) / n;

					memcpy(&new_mean_bits, &new_mean, sizeof(new_mean_bits));
				} while (!pce->mean_bits.compare_exchange(&old_mean_bits, new_mean_bits));

				// Atomically update M2 using bit-casting in a CAS loop
				uint32_t old_M2_bits;
				uint32_t new_M2_bits;
				float old_M2;
				do {
					old_M2_bits = pce->M2_bits.load();
					memcpy(&old_M2, &old_M2_bits, sizeof(old_M2));
					const float new_M2 = old_M2 + (dt - old_mean) * (dt - (old_mean + (dt - old_mean) / n));

					memcpy(&new_M2_bits, &new_M2, sizeof(new_M2_bits));
				} while (!pce->M2_bits.compare_exchange(&old_M2_bits, new_M2_bits));

				pce->time_start.store(0);
			}
		}
		break;

	default:
		break;
	}
}

void
perf_count_interval(perf_counter_t handle, hrt_abstime now)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t n = pci->event_count.fetch_add(1);

			if (n == 0) {
				pci->time_first.store(now);
				pci->time_last.store(now);
				return;
			}

			hrt_abstime last_time = pci->time_last.load();
			pci->time_last.store(now);


			if (last_time == 0) {
				return;
			}

			hrt_abstime interval = now - last_time;

			if (n == 1) {
				pci->time_least.store((uint32_t)interval);
				pci->time_most.store((uint32_t)interval);

				const float initial_mean = (float)interval / 1e6f;
				uint32_t initial_mean_bits;
				memcpy(&initial_mean_bits, &initial_mean, sizeof(initial_mean_bits));
				pci->mean_bits.store(initial_mean_bits);
				pci->M2_bits.store(0); // 0.0f bits are 0

			} else {
				uint32_t prev_least = pci->time_least.load();
				while ((uint32_t)interval < prev_least &&
				       !pci->time_least.compare_exchange(&prev_least, (uint32_t)interval));

				uint32_t prev_most = pci->time_most.load();
				while ((uint32_t)interval > prev_most &&
				       !pci->time_most.compare_exchange(&prev_most, (uint32_t)interval));

				float dt = (float)interval / 1e6f;
				uint32_t old_mean_bits;
				float old_mean;
				do {
					old_mean_bits = pci->mean_bits.load();
					memcpy(&old_mean, &old_mean_bits, sizeof(old_mean));
					const float new_mean = old_mean + (dt - old_mean) / n;
					uint32_t new_mean_bits;
					memcpy(&new_mean_bits, &new_mean, sizeof(new_mean_bits));
				} while (!pci->mean_bits.compare_exchange(&old_mean_bits, old_mean_bits));

				uint32_t old_M2_bits;
				uint32_t new_M2_bits;
				float old_M2;
				do {
					old_M2_bits = pci->M2_bits.load();
					memcpy(&old_M2, &old_M2_bits, sizeof(old_M2));
					const float new_M2 = old_M2 + (dt - old_mean) * (dt - (old_mean + (dt - old_mean) / n));

					memcpy(&new_M2_bits, &new_M2, sizeof(new_M2_bits));
				} while (!pci->M2_bits.compare_exchange(&old_M2_bits, new_M2_bits));
			}
			break;
		}

	default:
		break;
	}
}

void
perf_set_count(perf_counter_t handle, uint64_t count)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			((struct perf_ctr_count *)handle)->event_count.store(count);
		}
		break;

	default:
		break;
	}

}

void
perf_cancel(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->time_start.store(0);
		}
		break;

	default:
		break;
	}
}

void
perf_reset(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT:
		((struct perf_ctr_count *)handle)->event_count.store(0);
		break;

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			pce->event_count.store(0);
			pce->time_start.store(0);
			pce->time_total.store(0);
			pce->time_least.store(0);
			pce->time_most.store(0);
			pce->mean_bits.store(0);
			pce->M2_bits.store(0);
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			pci->event_count.store(0);
			pci->time_event.store(0);
			pci->time_first.store(0);
			pci->time_last.store(0);
			pci->time_least.store(0);
			pci->time_most.store(0);
			pci->mean_bits.store(0);
			pci->M2_bits.store(0);
			break;
		}
	}
}

void
perf_print_counter(perf_counter_t handle)
{
	if (handle == nullptr) {
		return;
	}

	switch (handle->type) {
	case PC_COUNT: {
			uint64_t event_count = ((struct perf_ctr_count *)handle)->event_count.load();
			PX4_INFO_RAW("%s: %" PRIu64 " events\n",
				     handle->name,
				     event_count);
			break;
		}

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint64_t event_count = pce->event_count.load();
			uint64_t time_total = pce->time_total.load();
			uint32_t time_least = pce->time_least.load();
			uint32_t time_most = pce->time_most.load();

			uint32_t M2_bits = pce->M2_bits.load();
			float M2;
			memcpy(&M2, &M2_bits, sizeof(M2));

			float rms = 0.0f;

			if (event_count > 1) {
				rms = sqrtf(M2 / (event_count - 1));
			}

			PX4_INFO_RAW("%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32
				     "us %5.3fus rms\n",
				     handle->name,
				     event_count,
				     time_total,
				     (event_count == 0) ? 0 : (double)time_total / (double)event_count,
				     time_least,
				     time_most,
				     (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t event_count = pci->event_count.load();
			uint64_t time_first = pci->time_first.load();
			uint64_t time_last = pci->time_last.load();
			uint32_t time_least = pci->time_least.load();
			uint32_t time_most = pci->time_most.load();

			uint32_t M2_bits = pci->M2_bits.load();
			float M2;
			memcpy(&M2, &M2_bits, sizeof(M2));

			float rms = 0.0f;

			if (event_count > 1) {
				rms = sqrtf(M2 / (event_count - 1));
			}

			PX4_INFO_RAW("%s: %" PRIu64 " events, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms\n",
				     handle->name,
				     event_count,
				     (event_count < 2) ? 0 : (double)(time_last - time_first) / (double)(event_count - 1),
				     time_least,
				     time_most,
				     (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}
}


int
perf_print_counter_buffer(char *buffer, int length, perf_counter_t handle)
{
	int num_written = 0;

	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT: {
			uint64_t event_count = ((struct perf_ctr_count *)handle)->event_count.load();
			num_written = snprintf(buffer, length, "%s: %" PRIu64 " events",
					       handle->name,
					       event_count);
			break;
		}

	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint64_t event_count = pce->event_count.load();
			uint64_t time_total = pce->time_total.load();
			uint32_t time_least = pce->time_least.load();
			uint32_t time_most = pce->time_most.load();

			uint32_t M2_bits = pce->M2_bits.load();
			float M2;
			memcpy(&M2, &M2_bits, sizeof(M2));

			float rms = 0.0f;

			if (event_count > 1) {
				rms = sqrtf(M2 / (event_count - 1));
			}

			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %" PRIu64 "us elapsed, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       event_count,
					       time_total,
					       (event_count == 0) ? 0 : (double)time_total / (double)event_count,
					       time_least,
					       time_most,
					       (double)(1e6f * rms));
			break;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint64_t event_count = pci->event_count.load();
			uint64_t time_first = pci->time_first.load();
			uint64_t time_last = pci->time_last.load();
			uint32_t time_least = pci->time_least.load();
			uint32_t time_most = pci->time_most.load();

			uint32_t M2_bits = pci->M2_bits.load();
			float M2;
			memcpy(&M2, &M2_bits, sizeof(M2));

			float rms = 0.0f;

			if (event_count > 1) {
				rms = sqrtf(M2 / (event_count - 1));
			}

			num_written = snprintf(buffer, length,
					       "%s: %" PRIu64 " events, %.2fus avg, min %" PRIu32 "us max %" PRIu32 "us %5.3fus rms",
					       handle->name,
					       event_count,
					       (event_count < 2) ? 0 : (double)(time_last - time_first) / (double)(event_count - 1),
					       time_least,
					       time_most,
					       (double)(1e6f * rms));
			break;
		}

	default:
		break;
	}

	if (num_written >= length) {
		num_written = length - 1;
	}

	buffer[num_written] = '\0';
	return num_written;
}

uint64_t
perf_event_count(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_COUNT:
		return ((struct perf_ctr_count *)handle)->event_count.load();

	case PC_ELAPSED: {
			return ((struct perf_ctr_elapsed *)handle)->event_count.load();
		}

	case PC_INTERVAL: {
			return ((struct perf_ctr_interval *)handle)->event_count.load();
		}

	default:
		break;
	}

	return 0;
}

float
perf_mean(perf_counter_t handle)
{
	if (handle == nullptr) {
		return 0;
	}

	switch (handle->type) {
	case PC_ELAPSED: {
			struct perf_ctr_elapsed *pce = (struct perf_ctr_elapsed *)handle;
			uint32_t mean_bits = pce->mean_bits.load();
			float mean_val;
			memcpy(&mean_val, &mean_bits, sizeof(mean_val));
			return mean_val;
		}

	case PC_INTERVAL: {
			struct perf_ctr_interval *pci = (struct perf_ctr_interval *)handle;
			uint32_t mean_bits = pci->mean_bits.load();
			float mean_val;
			memcpy(&mean_val, &mean_bits, sizeof(mean_val));
			return mean_val;
		}

	default:
		break;
	}

	return 0.0f;
}

void
perf_iterate_all(perf_callback cb, void *user)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		cb(handle, user);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_print_all(void)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		perf_print_counter(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);
}

void
perf_print_latency(void)
{
	latency_info_t latency;
	PX4_INFO_RAW("bucket [us] : events\n");

	for (int i = 0; i < get_latency_bucket_count(); i++) {
		latency = get_latency(i, i);
		PX4_INFO_RAW("        %4i : %li\n", latency.bucket, (long int)latency.counter);
	}

	// print the overflow bucket value
	latency = get_latency(get_latency_bucket_count() - 1, get_latency_bucket_count());
	PX4_INFO_RAW(" >%4" PRIu16 " : %" PRIu32 "\n", latency.bucket, latency.counter);
}

void
perf_reset_all(void)
{
	pthread_mutex_lock(&perf_counters_mutex);
	perf_counter_t handle = (perf_counter_t)sq_peek(&perf_counters);

	while (handle != nullptr) {
		perf_reset(handle);
		handle = (perf_counter_t)sq_next(&handle->link);
	}

	pthread_mutex_unlock(&perf_counters_mutex);

	reset_latency_counters();
}
