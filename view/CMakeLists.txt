add_executable(MyApp main.cpp)

include(FetchContent)

FetchContent_Declare(
	matplot
	GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus.git
	GIT_TAG master
)

FetchContent_MakeAvailable(matplot)

target_link_libraries(MyApp PRIVATE
	MyLib
	eigen
	matplot
)