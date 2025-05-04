# ќпредел€ем переменную EIGEN_ROOT_DIR, которую пользователь может задать
option(EIGEN_USE_LOCAL "Use local installation of Eigen library" OFF)
if(EIGEN_USE_LOCAL)
    message(STATUS "Using local Eigen installation from ${EIGEN_ROOT_DIR}")
else()
    message(STATUS "Fetching Eigen library")
endif()

# ѕеред проверкой наличи€ локального пути задаЄм значение по умолчанию
if(NOT DEFINED EIGEN_ROOT_DIR OR NOT EXISTS "${EIGEN_ROOT_DIR}/eigen3/Eigen/Dense")
    cmake_minimum_required(VERSION 3.10)

    project(A_posteriory_analysis)

    include(FetchContent)

    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG master  # ћожно выбрать стабильную версию
    )

    FetchContent_MakeAvailable(eigen)
else()
    add_subdirectory("${EIGEN_ROOT_DIR}" eigen_external)
    list(APPEND CMAKE_PREFIX_PATH "${EIGEN_ROOT_DIR}")
endif()

# ƒалее добавл€ем остальные команды
add_subdirectory(lib)
add_subdirectory(view)