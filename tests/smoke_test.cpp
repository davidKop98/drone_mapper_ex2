#include <gtest/gtest.h>

// Trivial smoke test: confirms the test target builds, links GTest, and is
// discoverable by ctest / --gtest_filter. Real component and integration
// suites will be added under tests/components and tests/integration.
TEST(Smoke, Builds) {
    EXPECT_TRUE(true);
}
