#include "tests.h"

#define TEST_PASS_ENABLE_IS_ENABLED(test_text) \
	"Test " test_text " enable, is enabled...PASS\r\n"

#define TEST_PASS_ENABLE_NOT_ENABLED(test_text) \
	"Test " test_text " enable, not enabled...PASS\r\n"

#define TEST_PASS_MASK_DISABLE_NOT_ENABLED(test_text) \
	"Test " test_text " mask disable, not enabled...PASS\r\n"

#define TEST_PASS_MASK_DISABLE_IS_ENABLED(test_text) \
	"Test " test_text " mask disable, is enabled...PASS\r\n"

#define	TEST_PASS_DISABLE_WDCE_WDE_NOT_ENABLED(test_text) \
	"Test WDCE and WDE disable, " test_text " not enabled...PASS\r\n"

int main(int argc, char **argv) {
	tests_init(argc, argv);

	static const char *expected =
		TEST_PASS_ENABLE_IS_ENABLED("WDIE")
		TEST_PASS_MASK_DISABLE_NOT_ENABLED("WDIE")
		TEST_PASS_ENABLE_NOT_ENABLED("WDE")
		"Test WDP change, not changed...PASS\r\n"
		"Test WDCE and WDP change...PASS\r\n"
		"Test WDCE and WDE enable...PASS\r\n"
		TEST_PASS_MASK_DISABLE_IS_ENABLED("WDE")
		TEST_PASS_DISABLE_WDCE_WDE_NOT_ENABLED("WDE");

	tests_assert_uart_receive("atmega48_watchdog_register_test.axf", 1000000,
				  expected, '0');
	tests_success();
	return 0;
}
