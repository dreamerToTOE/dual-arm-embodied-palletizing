// Isaac 4.5 exposes a CoreNodes argument without exporting its Python class.
#include <pybind11/pybind11.h>
#include <omni/fabric/core/RationalTime.h>

PYBIND11_MODULE(_r1_fabric, m) {
    using omni::fabric::RationalTime;
    pybind11::class_<RationalTime>(m, "RationalTime")
        .def(pybind11::init<int64_t, uint64_t>())
        .def_readwrite("numerator", &RationalTime::numerator)
        .def_readwrite("denominator", &RationalTime::denominator);
}
