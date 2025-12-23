#include <unity.h>

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_example(void) {
    TEST_ASSERT_EQUAL(1, 1);
}

int main( int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_example);
    UNITY_END();
}