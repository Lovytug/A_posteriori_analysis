add_executable(MyApp main.cpp)

target_link_libraries(MyApp PRIVATE
	MyLib
	eigen
)