/*
* Copyright 2009 Free Software Foundation, Inc.
* Copyright 2010 Kestrel Signal Processing, Inc.
*
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <iostream>
#include <iterator>

#include "Logger.h"
extern "C" {
#include <osmocom/core/msgb.h>
#include <osmocom/core/talloc.h>
#include <osmocom/core/application.h>
#include <osmocom/core/utils.h>
#include "debug.h"
}

#define MYCAT 0

int main(int argc, char *argv[])
{
	struct log_info_cat categories[1];
	struct log_info linfo;
	categories[MYCAT] = {
		"MYCAT",
		NULL,
		"Whatever",
		LOGL_NOTICE,
		1,
	};
	linfo.cat = categories;
	linfo.num_cat = ARRAY_SIZE(categories);

	void *tall_ctx = talloc_named_const(NULL, 1, "OsmoTRX context");
	msgb_talloc_ctx_init(tall_ctx, 0);

	osmo_init_logging2(tall_ctx, &linfo);

	log_set_use_color(osmo_stderr_target, 0);
	log_set_print_filename(osmo_stderr_target, 0);
	log_set_print_level(osmo_stderr_target, 1);

	Log(MYCAT, LOGL_FATAL).get() << "testing the logger.";
	Log(MYCAT, LOGL_ERROR).get() << "testing the logger.";
	Log(MYCAT, LOGL_NOTICE).get() << "testing the logger.";
	Log(MYCAT, LOGL_INFO).get() << "testing the logger.";
	Log(MYCAT, LOGL_DEBUG).get() << "testing the logger.";
}
