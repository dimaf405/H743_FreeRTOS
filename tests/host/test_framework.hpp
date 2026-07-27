#pragma once

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace host_test {

using TestFunction = void (*)();

struct TestCase {
    const char *name;
    TestFunction function;
};

inline std::vector<TestCase> &registry()
{
    static std::vector<TestCase> tests;
    return tests;
}

class Registrar {
public:
    Registrar(const char *name, TestFunction function)
    {
        registry().push_back({name, function});
    }
};

inline void check(bool condition, const char *expression, const char *file, int line)
{
    if (!condition) {
        throw std::runtime_error(std::string(file) + ":" + std::to_string(line) +
                                 ": assertion failed: " + expression);
    }
}

inline int run_all()
{
    int failures = 0;
    for (const auto &test : registry()) {
        try {
            test.function();
            std::cout << "PASS " << test.name << '\n';
        } catch (const std::exception &error) {
            ++failures;
            std::cerr << "FAIL " << test.name << ": " << error.what() << '\n';
        } catch (...) {
            ++failures;
            std::cerr << "FAIL " << test.name << ": unknown exception\n";
        }
    }
    std::cout << registry().size() << " test(s), " << failures << " failure(s)\n";
    return failures == 0 ? 0 : 1;
}

} // namespace host_test

#define HOST_TEST(name) \
    static void name(); \
    static ::host_test::Registrar registrar_##name(#name, &name); \
    static void name()

#define CHECK(expression) ::host_test::check((expression), #expression, __FILE__, __LINE__)
#define CHECK_EQ(left, right) CHECK((left) == (right))
#define CHECK_NE(left, right) CHECK((left) != (right))
