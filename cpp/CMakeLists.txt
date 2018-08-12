cmake_minimum_required(VERSION 2.8.12)
set(bindings_python_version 2.7)

project(PathAdaptiveCore)

add_subdirectory(pybind11)
pybind11_add_module(PathAdaptiveCore bindings.cpp adaptive.cpp clipper.cpp)

