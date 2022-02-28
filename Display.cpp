

#include <chrono>
#include <thread>
#include "../gl.h"
#include "Display.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <external/stb/stb_image.h>
#include <stdio.h>
#include <stdlib.h>
#include "igl/igl_inline.h"
#include <igl/get_seconds.h>
#include "igl/opengl/glfw/renderer.h"
#include <string>
#include <fstream>
#include <sstream>

static void glfw_error_callback(int error, const char* description)
{
	fputs(description, stderr);
}

Display::Display(int windowWidth, int windowHeight, const std::string& title)
{

	bool resizable = true, fullscreen = false;
		glfwSetErrorCallback(glfw_error_callback);
		if (!glfwInit())
		{
			exit(EXIT_FAILURE);
		}
		glfwWindowHint(GLFW_SAMPLES, 8);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		
//#ifdef __APPLE__
//		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
//#endif
//		if (fullscreen)
//		{
//			GLFWmonitor* monitor = glfwGetPrimaryMonitor();
//			const GLFWvidmode* mode = glfwGetVideoMode(monitor);
//			window = glfwCreateWindow(mode->width, mode->height, title.c_str(), monitor, nullptr);
//			windowWidth = mode->width;
//			windowHeight = mode->height;
//		}
//		else
//		{
			// Set default windows width
			//if (windowWidth <= 0 & core_list.size() == 1 && renderer->core().viewport[2] > 0)
			//	windowWidth = renderer->core().viewport[2];
			//else 
			//	if (windowWidth <= 0)
			//	windowWidth = 1280;
			//// Set default windows height
			//if (windowHeight <= 0 & core_list.size() == 1 && renderer->core().viewport[3] > 0)
			//	windowHeight = renderer->core().viewport[3];
			//else if (windowHeight <= 0)
			//	windowHeight = 800;
//			window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
//		}
		window = glfwCreateWindow(windowWidth, windowHeight, title.c_str(), nullptr, nullptr);
		if (!window)
		{
			glfwTerminate();
			exit(EXIT_FAILURE);
		}
		glfwMakeContextCurrent(window);//??

		// Load OpenGL and its extensions
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			printf("Failed to load OpenGL and its extensions\n");
			exit(EXIT_FAILURE);
		}
//#if defined(DEBUG) || defined(_DEBUG)
//		printf("OpenGL Version %d.%d loaded\n", GLVersion.major, GLVersion.minor);
//		int major, minor, rev;
//		major = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MAJOR);
//		minor = glfwGetWindowAttrib(window, GLFW_CONTEXT_VERSION_MINOR);
//		rev = glfwGetWindowAttrib(window, GLFW_CONTEXT_REVISION);
//		printf("OpenGL version received: %d.%d.%d\n", major, minor, rev);
//		printf("Supported OpenGL is %s\n", (const char*)glGetString(GL_VERSION));
//		printf("Supported GLSL is %s\n", (const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));
//#endif
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
		//Tamir: changes from here
		// Initialize FormScreen
	   // __viewer = this;
		// Register callbacks
		//glfwSetKeyCallback(window, glfw_key_callback);
		//glfwSetCursorPosCallback(window,glfw_mouse_move);
		//glfwSetScrollCallback(window, glfw_mouse_scroll);
		//glfwSetMouseButtonCallback(window, glfw_mouse_press);
		//glfwSetWindowSizeCallback(window,glfw_window_size);
	
		
		//glfwSetCharModsCallback(window,glfw_char_mods_callback);
		//glfwSetDropCallback(window,glfw_drop_callback);
		// Handle retina displays (windows and mac)
		//int width, height;
		//glfwGetFramebufferSize(window, &width, &height);
		//int width_window, height_window;
		//glfwGetWindowSize(window, &width_window, &height_window);
		//highdpi = windowWidth/width_window;
		
		//glfw_window_size(window,width_window,height_window);
		//opengl.init();
//		core().align_camera_center(data().V, data().F);
		// Initialize IGL viewer
//		init();

		
}
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

bool Display::launch_rendering(bool loop)
{
	
	float skyboxVertices[] = {
		// positions          
		-1.0f,  1.0f, -1.0f,
		-1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f, -1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,

		-1.0f, -1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f,
		-1.0f, -1.0f,  1.0f,

		-1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f, -1.0f,
		 1.0f,  1.0f,  1.0f,
		 1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f,  1.0f,
		-1.0f,  1.0f, -1.0f,

		-1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f, -1.0f,
		 1.0f, -1.0f, -1.0f,
		-1.0f, -1.0f,  1.0f,
		 1.0f, -1.0f,  1.0f
	};
	for (int i = 0; i < 6 * 18; i++) {
		skyboxVertices[i] *= 50;
		
	}
    //glfwMakeContextCurrent(window);
	//glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);//???

	// glad: load all OpenGL function pointers
	// ---------------------------------------
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	// configure global opengl state
// -----------------------------
	glEnable(GL_DEPTH_TEST);

	
	//shader
	unsigned int ID;
	const char* vertexPath = "C:/Users/97252/Desktop/courses/3D_animation/project/skybox.vs";
	const char* fragmentPath = "C:/Users/97252/Desktop/courses/3D_animation/project/skybox.fs";
	// 1. retrieve the vertex/fragment source code from filePath
	std::string vertexCode;
	std::string fragmentCode;
	std::ifstream vShaderFile;
	std::ifstream fShaderFile;
	// ensure ifstream objects can throw exceptions:
	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// open files
		vShaderFile.open(vertexPath);
		fShaderFile.open(fragmentPath);
		std::stringstream vShaderStream, fShaderStream;
		// read file's buffer contents into streams
		vShaderStream << vShaderFile.rdbuf();
		fShaderStream << fShaderFile.rdbuf();
		// close file handlers
		vShaderFile.close();
		fShaderFile.close();
		// convert stream into string
		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();
	
	}
	catch (std::ifstream::failure& e)
	{
		std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
	}
	const char* vShaderCode = vertexCode.c_str();
	const char* fShaderCode = fragmentCode.c_str();
	// 2. compile shaders
	unsigned int vertex, fragment;
	// vertex shader
	vertex = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex, 1, &vShaderCode, NULL);
	glCompileShader(vertex);

	// fragment Shader
	fragment = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment, 1, &fShaderCode, NULL);
	glCompileShader(fragment);

	// shader Program
	ID = glCreateProgram();
	glAttachShader(ID, vertex);
	glAttachShader(ID, fragment);
	glLinkProgram(ID);

	// delete the shaders as they're linked into our program now and no longer necessary
	glDeleteShader(vertex);
	glDeleteShader(fragment);
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

	unsigned int cubemapTexture = loadCubeMap();
	glUseProgram(ID);
	glUniform1i(glGetUniformLocation(ID, "skybox"), 0); // set int
	// Rendering loop
	const int num_extra_frames = 5;
	int frame_counter = 0;
	int windowWidth, windowHeight;
	//main loop
	Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);

	glfwGetWindowSize(window, &windowWidth, &windowHeight);
	renderer->post_resize(window, windowWidth, windowHeight);
	for (int i = 0; i < renderer->GetScene()->data_list.size(); i++) {
		renderer->core().toggle(renderer->GetScene()->data_list[i].show_lines);
	}
	
	
	while (!glfwWindowShouldClose(window))
	{
		
		double tic = igl::get_seconds();


		
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		renderer->Animate();
		renderer->draw(window);
		glDepthFunc(GL_LEQUAL);

		glUseProgram(ID);
		// skybox cube
		Eigen::Matrix4f view = renderer->core().view;
		view.col(3) = Eigen::Vector4f(0, 0, 0, 1);
		Eigen::Matrix4f proj = renderer->core().proj;
		glUniformMatrix4fv(glGetUniformLocation(ID, "view"), 1, GL_FALSE, view.data());
		glUniformMatrix4fv(glGetUniformLocation(ID, "projection"), 1, GL_FALSE, proj.data());
		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		//glDrawElements(GL_TRIANGLES, sizeof(skyboxVertices) / sizeof(GLushort), GL_UNSIGNED_SHORT, 0);
		glDepthFunc(GL_LESS); // set depth function back to default

		glfwSwapBuffers(window);



		if (renderer->core().is_animating || frame_counter++ < num_extra_frames)
		{//motion
			glfwPollEvents();
			// In microseconds
			double duration = 1000000. * (igl::get_seconds() - tic);
			const double min_duration = 1000000. / renderer->core().animation_max_fps;
			if (duration < min_duration)
			{
				std::this_thread::sleep_for(std::chrono::microseconds((int)(min_duration - duration)));
			}
		}
		else
		{
			glfwPollEvents();
			frame_counter = 0;
		}
		if (!loop)
			return !glfwWindowShouldClose(window);

#ifdef __APPLE__
		static bool first_time_hack = true;
		if (first_time_hack) {
			glfwHideWindow(window);
			glfwShowWindow(window);
			first_time_hack = false;
		}
#endif
	}
	return EXIT_SUCCESS;
}

void Display::AddKeyCallBack(void(*keyCallback)(GLFWwindow*, int, int, int, int))
{
	glfwSetKeyCallback(window, (void(*)(GLFWwindow*, int, int, int, int))keyCallback);//{

}

void Display::AddMouseCallBacks(void (*mousebuttonfun)(GLFWwindow*, int, int, int), void (*scrollfun)(GLFWwindow*, double, double), void (*cursorposfun)(GLFWwindow*, double, double))
{
	glfwSetMouseButtonCallback(window, mousebuttonfun);
	glfwSetScrollCallback(window, scrollfun);
	glfwSetCursorPosCallback(window, cursorposfun);
}

void Display::AddResizeCallBack(void (*windowsizefun)(GLFWwindow*, int, int))
{
	glfwSetWindowSizeCallback(window, windowsizefun);
}

void Display::SetRenderer(void* userPointer)
{
	
	glfwSetWindowUserPointer(window, userPointer);
	
}

void* Display::GetScene()
{
	return glfwGetWindowUserPointer(window);
}

void Display::SwapBuffers()
{
	glfwSwapBuffers(window);
}

void Display::PollEvents()
{
	glfwPollEvents();
}

int Display::loadCubeMap() {
	std::vector<std::string> faces
	{
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Right.bmp",
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Left.bmp",
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Top.bmp",
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Bottom.bmp",
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Front.bmp",
		"C:/Users/97252/Desktop/courses/3D_animation/project/Daylight_Box_Pieces/Box_Back.bmp"
	};
	unsigned int textureID;
	glGenTextures(1, &textureID); 
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
	int width, height, nrChannels;
	unsigned char* data;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data) {
			glTexImage2D(
				GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
				0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
			);
			stbi_image_free(data);
		}
		else {
			std::cout << "Cubemap tex failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	return textureID;
}

Display::~Display()
{
	glfwDestroyWindow(window);
	glfwTerminate();
}

