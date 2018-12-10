#include <assert.h>
#include <lime/LimeSuite.h>
#include <string>

extern "C"
{
size_t osmo_strlcpy(char *dst, const char *src, size_t siz);
}

int info_list_find(lms_info_str_t* info_list, unsigned int count, const std::string &args);

using namespace std;

int main(void)
{
	unsigned int count;
	lms_info_str_t* info_list;
	std::string args;

	/* two fake entries for info_list */
	count = 2;
	info_list = new lms_info_str_t[count];
	osmo_strlcpy(info_list[0], "LimeSDR Mini, addr=24607:1337, serial=FAKESERIAL0001", sizeof(lms_info_str_t));
	osmo_strlcpy(info_list[1], "LimeSDR Mini, addr=24607:1338, serial=FAKESERIAL0002", sizeof(lms_info_str_t));

	/* find second entry by args filter */
	args = "serial=FAKESERIAL0002,LimeSDR Mini";
	assert(info_list_find(info_list, count, args) == 1);

	/* empty args -> first entry */
	args = "";
	assert(info_list_find(info_list, count, args) == 0);

	/* not matching args -> -1 */
	args = "serial=NOTMATCHING";
	assert(info_list_find(info_list, count, args) == -1);

	/* clean up */
	delete[] info_list;
	return 0;
}
