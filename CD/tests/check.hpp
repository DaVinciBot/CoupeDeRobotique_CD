#pragma once
// Mini-bibliothèque de test (aucune dépendance) : CHECK, CHECK_EQ, CHECK_NEAR, RUN, finish().
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

namespace testing_util {

inline int& checks() { static int n = 0; return n; }
inline int& failures() { static int n = 0; return n; }

template <class A, class B>
inline void checkEq(const A& a, const B& b, const char* ea, const char* eb, const char* file, int line) {
    ++checks();
    if (!(a == b)) {
        ++failures();
        std::ostringstream os;
        os << a << "  !=  " << b;
        std::fprintf(stderr, "  ECHEC %s:%d : %s == %s   (%s)\n", file, line, ea, eb, os.str().c_str());
    }
}

inline void checkNear(double a, double b, double tol, const char* ea, const char* eb, const char* file, int line) {
    ++checks();
    if (!(std::fabs(a - b) <= tol)) {
        ++failures();
        std::fprintf(stderr, "  ECHEC %s:%d : |%s - %s| <= %g   (%.4f vs %.4f)\n", file, line, ea, eb, tol, a, b);
    }
}

inline int finish(const char* name) {
    std::printf("\n%s : %d vérifications, %d échec(s)\n", name, checks(), failures());
    return failures() == 0 ? 0 : 1;
}

}  // namespace testing_util

#define CHECK(cond)                                                                                 \
    do {                                                                                            \
        ++testing_util::checks();                                                                   \
        if (!(cond)) {                                                                              \
            ++testing_util::failures();                                                             \
            std::fprintf(stderr, "  ECHEC %s:%d : %s\n", __FILE__, __LINE__, #cond);                \
        }                                                                                           \
    } while (0)

#define CHECK_EQ(a, b) testing_util::checkEq((a), (b), #a, #b, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) testing_util::checkNear((a), (b), (tol), #a, #b, __FILE__, __LINE__)

// Lance une fonction de test en affichant son nom.
#define RUN(fn)                      \
    do {                             \
        std::printf("[%s]\n", #fn);  \
        fn();                        \
    } while (0)
