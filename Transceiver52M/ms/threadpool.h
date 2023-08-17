#pragma once
/*
 * (C) 2023 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
 * All Rights Reserved
 *
 * Author: Eric Wild <ewild@sysmocom.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <future>
#include <mutex>
#include <queue>

struct single_thread_pool {
	std::mutex m;
	std::condition_variable cv;
	std::atomic<bool> stop_flag;
	std::atomic<bool> is_ready;
	std::deque<std::function<void()>> wq;
	std::thread worker_thread;

	template <class F>
	void add_task(F &&f)
	{
		std::unique_lock<std::mutex> l(m);
		wq.emplace_back(std::forward<F>(f));
		cv.notify_one();
		return;
	}

	single_thread_pool() : stop_flag(false), is_ready(false), worker_thread(std::thread([this] { thread_loop(); }))
	{
	}
	~single_thread_pool()
	{
		stop();
	}

	std::thread::native_handle_type get_handle()
	{
		return worker_thread.native_handle();
	}

    private:
	void stop()
	{
		{
			std::unique_lock<std::mutex> l(m);
			wq.clear();
			stop_flag = true;
			cv.notify_one();
		}
		worker_thread.join();
	}

	void thread_loop()
	{
		while (true) {
			is_ready = true;
			std::function<void()> f;
			{
				std::unique_lock<std::mutex> l(m);
				if (wq.empty()) {
					cv.wait(l, [&] { return !wq.empty() || stop_flag; });
				}
				if (stop_flag)
					return;
				is_ready = false;
				f = std::move(wq.front());
				wq.pop_front();
			}
			f();
		}
	}
};