cmake_minimum_required(VERSION 3.10)
project(py_acl_serial_driver)

set(PYPKG_DIR "${CMAKE_CURRENT_BINARY_DIR}/acl_serial_driver")

pybind11_add_module(py_acl_serial_driver py_acl_serial_driver.cpp)
target_link_libraries(py_acl_serial_driver PUBLIC acl_serial_driver)
set_target_properties(py_acl_serial_driver
  PROPERTIES
    OUTPUT_NAME "acl_serial_driver"
    LIBRARY_OUTPUT_DIRECTORY "${PYPKG_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYPKG_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PYPKG_DIR}"
)

# copy setup.py file binary dir for install with: pip install .
configure_file(setup.py.in ${CMAKE_CURRENT_BINARY_DIR}/setup.py)

# Create the Python package -- Note that "." is used to conform to PEP 328
file(WRITE "${PYPKG_DIR}/__init__.py"
    "from .acl_serial_driver import *\n"
    "from .acl_serial_driver import __version__\n"
    "from .acl_serial_driver import __doc__")