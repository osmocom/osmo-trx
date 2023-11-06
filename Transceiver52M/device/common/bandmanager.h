#pragma once
/*
 * (C) 2022 by sysmocom s.f.m.c. GmbH <info@sysmocom.de>
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

#include <string>
#include <tuple>

#include "Logger.h"

extern "C" {
#include <osmocom/gsm/gsm_utils.h>
}

template <typename powermapt, typename devmapt>
class band_manager {
	using powerkeyt = typename powermapt::key_type;
	using powermappedt = typename powermapt::mapped_type;
	using devkeyt = typename devmapt::key_type;
	devkeyt m_dev_type;
	const powermapt &m_power_map;
	const devmapt &m_dev_map;
	powerkeyt m_fallback;
	enum gsm_band m_band;
	powermappedt m_band_desc;
	bool band_ass_curr_sess{}; /* true if  "band" was set after last POWEROFF */

	// looks up either first tuple element (->enum) or straight enum
	template <typename T, typename std::enable_if<std::is_enum<T>::value>::type *dummy = nullptr>
	auto key_helper(T &t) -> T
	{
		return t;
	}

	template <typename T>
	auto key_helper(T t) -> typename std::tuple_element<0, T>::type
	{
		return std::get<0>(t);
	}

	void assign_band_desc(enum gsm_band req_band)
	{
		auto key = key_helper(m_dev_type);
		auto fallback_key = key_helper(m_fallback);
		auto it = m_power_map.find({ key, req_band });
		if (it == m_power_map.end()) {
			auto desc = m_dev_map.at(m_dev_type);
			LOGC(DDEV, ERROR) << "No Tx Power measurements exist for device " << desc.desc_str
					  << " on band " << gsm_band_name(req_band) << ", using fallback..";
			it = m_power_map.find({ fallback_key, req_band });
		}
		OSMO_ASSERT(it != m_power_map.end());
		m_band_desc = it->second;
	}

	bool set_band(enum gsm_band req_band)
	{
		if (band_ass_curr_sess && req_band != m_band) {
			LOGC(DDEV, ALERT) << "Requesting band " << gsm_band_name(req_band)
					  << " different from previous band " << gsm_band_name(m_band);
			return false;
		}

		if (req_band != m_band) {
			m_band = req_band;
			assign_band_desc(m_band);
		}
		band_ass_curr_sess = true;
		return true;
	}

    public:
	band_manager(const devkeyt &dev_type, const powermapt &power_map, const devmapt &dev_map, powerkeyt fallback)
		: m_dev_type(dev_type), m_power_map(power_map), m_dev_map(dev_map), m_fallback(fallback),
		  m_band((enum gsm_band)0)
	{
	}
	band_manager(const powermapt &power_map, const devmapt &dev_map)
		: m_dev_type(dev_map.begin()->first), m_power_map(power_map), m_dev_map(dev_map),
		  m_fallback(m_power_map.begin()->first), m_band((enum gsm_band)0)
	{
	}
	void band_reset()
	{
		band_ass_curr_sess = false;
	}

	void update_band_dev(devkeyt dev_type) {
		m_dev_type = dev_type;
	}

	void get_dev_band_desc(powermappedt &desc)
	{
		if (m_band == 0) {
			LOGC(DDEV, ERROR)
				<< "Power parameters requested before Tx Frequency was set! Providing band 900 by default...";
			assign_band_desc(GSM_BAND_900);
		}
		desc = m_band_desc;
	}

	bool update_band_from_freq(double wFreq, int chan, bool is_tx)
	{
		enum gsm_band req_band;
		auto dirstr = is_tx ? "Tx" : "Rx";
		auto req_arfcn = gsm_freq102arfcn(wFreq / 1000 / 100, !is_tx);
		if (req_arfcn == 0xffff) {
			LOGCHAN(chan, DDEV, ALERT)
				<< "Unknown ARFCN for " << dirstr << " Frequency " << wFreq / 1000 << " kHz";
			return false;
		}
		if (gsm_arfcn2band_rc(req_arfcn, &req_band) < 0) {
			LOGCHAN(chan, DDEV, ALERT) << "Unknown GSM band for " << dirstr << " Frequency " << wFreq
						   << " Hz (ARFCN " << req_arfcn << " )";
			return false;
		}

		return set_band(req_band);
	}
};