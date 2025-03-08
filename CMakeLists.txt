cmake_minimum_required(VERSION 3.10)

project(A_posteriory_analysis)

include(FetchContent)

FetchContent_Declare(
	eigen
	GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
	GIT_TAG master
)

FetchContent_MakeAvailable(eigen)


add_subdirectory(lib)