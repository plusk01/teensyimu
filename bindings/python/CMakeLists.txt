set(PYPKG_DIR "${CMAKE_CURRENT_BINARY_DIR}/teensyimu")

pybind11_add_module(py_teensyimu py_teensyimu.cpp)
target_link_libraries(py_teensyimu PUBLIC teensyimu)
set_target_properties(py_teensyimu
  PROPERTIES
    OUTPUT_NAME "teensyimu"
    LIBRARY_OUTPUT_DIRECTORY "${PYPKG_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PYPKG_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PYPKG_DIR}"
)

# copy setup.py file binary dir for install with: pip install .
configure_file(setup.py.in ${CMAKE_CURRENT_BINARY_DIR}/setup.py)

# copy any python files over to the module
add_custom_command(TARGET py_teensyimu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_CURRENT_SOURCE_DIR}/tools.py $<TARGET_FILE_DIR:py_teensyimu>)

add_custom_command(TARGET py_teensyimu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/plotimu.py $<TARGET_FILE_DIR:py_teensyimu>)

add_custom_command(TARGET py_teensyimu POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
                ${CMAKE_CURRENT_SOURCE_DIR}/../../examples/plotfreq.py $<TARGET_FILE_DIR:py_teensyimu>)

# Create the Python package -- Note that "." is used to conform to PEP 328
file(WRITE "${PYPKG_DIR}/__init__.py"
    "from .teensyimu import *\n"
    "from .teensyimu import __version__\n"
    "from .teensyimu import __doc__\n\n"
    "from . import tools")

set(DIST "none")
if(UNIX AND NOT APPLE)
    execute_process(COMMAND bash -c "lsb_release -cs" OUTPUT_VARIABLE UBUNTU_DIST)
    string(STRIP "${UBUNTU_DIST}" UBUNTU_DIST)
    set(DIST "${UBUNTU_DIST}")
elseif(APPLE)
    set(DIST "macos")
elseif(WIN32)
    set(DIST "win10")
endif()

set(PKGSTR teensyimu-py3-${DIST}-${PROJECT_VERSION})
add_custom_target(pypkg
    DEPENDS py_teensyimu
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PKGSTR}
    COMMAND ${CMAKE_COMMAND} -E copy setup.py ${PKGSTR}/
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${PYPKG_DIR} ${PKGSTR}/teensyimu
    COMMAND ${CMAKE_COMMAND} -E tar zcvf ${PKGSTR}.tar.gz ${PKGSTR}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

add_custom_target(pip-install
    DEPENDS pypkg
    COMMAND ${PYTHON_EXECUTABLE} -m pip install .
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PKGSTR}
)