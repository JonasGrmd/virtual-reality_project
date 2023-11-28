#include<iostream>

//include glad before GLFW to avoid header conflict or define "#define GLFW_INCLUDE_NONE"
#include <glad/glad.h>
#include <GLFW/glfw3.h>

//include bullet
#include <src/btBulletCollisionCommon.h>
#include <src/btBulletDynamicsCommon.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


#include "./../Headers/camera.h"
#include "./../Headers/shader.h"
#include "./../Headers/object.h"

const int width = 1920;
const int height = 1080;


GLuint compileShader(std::string shaderCode, GLenum shaderType);
GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader);
void processInput(GLFWwindow* window);


#ifndef NDEBUG
void APIENTRY glDebugOutput(GLenum source,
	GLenum type,
	unsigned int id,
	GLenum severity,
	GLsizei length,
	const char* message,
	const void* userParam)
{
	// ignore non-significant error/warning codes
	if (id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

	std::cout << "---------------" << std::endl;
	std::cout << "Debug message (" << id << "): " << message << std::endl;

	switch (source)
	{
	case GL_DEBUG_SOURCE_API:             std::cout << "Source: API"; break;
	case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   std::cout << "Source: Window System"; break;
	case GL_DEBUG_SOURCE_SHADER_COMPILER: std::cout << "Source: Shader Compiler"; break;
	case GL_DEBUG_SOURCE_THIRD_PARTY:     std::cout << "Source: Third Party"; break;
	case GL_DEBUG_SOURCE_APPLICATION:     std::cout << "Source: Application"; break;
	case GL_DEBUG_SOURCE_OTHER:           std::cout << "Source: Other"; break;
	} std::cout << std::endl;

	switch (type)
	{
	case GL_DEBUG_TYPE_ERROR:               std::cout << "Type: Error"; break;
	case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: std::cout << "Type: Deprecated Behaviour"; break;
	case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  std::cout << "Type: Undefined Behaviour"; break;
	case GL_DEBUG_TYPE_PORTABILITY:         std::cout << "Type: Portability"; break;
	case GL_DEBUG_TYPE_PERFORMANCE:         std::cout << "Type: Performance"; break;
	case GL_DEBUG_TYPE_MARKER:              std::cout << "Type: Marker"; break;
	case GL_DEBUG_TYPE_PUSH_GROUP:          std::cout << "Type: Push Group"; break;
	case GL_DEBUG_TYPE_POP_GROUP:           std::cout << "Type: Pop Group"; break;
	case GL_DEBUG_TYPE_OTHER:               std::cout << "Type: Other"; break;
	} std::cout << std::endl;

	switch (severity)
	{
	case GL_DEBUG_SEVERITY_HIGH:         std::cout << "Severity: high"; break;
	case GL_DEBUG_SEVERITY_MEDIUM:       std::cout << "Severity: medium"; break;
	case GL_DEBUG_SEVERITY_LOW:          std::cout << "Severity: low"; break;
	case GL_DEBUG_SEVERITY_NOTIFICATION: std::cout << "Severity: notification"; break;
	} std::cout << std::endl;
	std::cout << std::endl;
}
#endif

Camera camera(glm::vec3(0.0, 10.0, 50.0));

btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionConfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;

btRigidBody* addSphere(float rad, float x, float y, float z, float mass) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));
	btSphereShape* sphere = new btSphereShape(rad);
	btVector3 inertia(btVector3(0.0, 0.0, 0.0));
	if (mass != 0.0) sphere->calculateLocalInertia(mass, inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, sphere, inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	return body;	
}

void init() {
	//Initialize the bullet world
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0.0, -10.0, 0.0));

	//Creating a static plane for the ground
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(0.0,0.0,0.0));
	btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0.0,1.0,0.0),0.0);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(0.0, motion, plane);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
};

int main(int argc, char* argv[])
{
	//Initialize Bullet world
	init();
	//Boilerplate
	//Create the OpenGL context 
	if (!glfwInit()) {
		throw std::runtime_error("Failed to initialise GLFW \n");
	}
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifndef NDEBUG
	//create a debug context to help with Debugging
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, true);
#endif


	//Create the window
	GLFWwindow* window = glfwCreateWindow(width, height, "Exercise 07", nullptr, nullptr);
	if (window == NULL)
	{
		glfwTerminate();
		throw std::runtime_error("Failed to create GLFW window\n");
	}

	glfwMakeContextCurrent(window);
	
	//load openGL function
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		throw std::runtime_error("Failed to initialize GLAD");
	}

	glEnable(GL_DEPTH_TEST);

#ifndef NDEBUG
	int flags;
	glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
	if (flags & GL_CONTEXT_FLAG_DEBUG_BIT)
	{
		glEnable(GL_DEBUG_OUTPUT);
		glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
		glDebugMessageCallback(glDebugOutput, nullptr);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	}
#endif

	//Initialize the bullet world

	const std::string sourceV = "#version 330 core\n"
		"in vec3 position; \n"
		"in vec2 tex_coords; \n"
		"in vec3 normal; \n"

		"out vec3 v_frag_coord; \n"
		"out vec3 v_normal; \n"

		"uniform mat4 M; \n"
		"uniform mat4 itM; \n"
		"uniform mat4 V; \n"
		"uniform mat4 P; \n"


		" void main(){ \n"
		"vec4 frag_coord = M*vec4(position, 1.0); \n"
		"gl_Position = P*V*frag_coord; \n"
		
		"v_normal = normalize(vec3(itM * vec4(normal, 1.0))); \n"
		"v_frag_coord = frag_coord.xyz; \n"
		"\n" 
		"}\n";

	const std::string sourceF = "#version 330 core\n"
		"out vec4 FragColor;\n"
		"precision mediump float; \n"

		"in vec3 v_frag_coord; \n"
		"in vec3 v_normal; \n"

		"uniform vec3 u_view_pos; \n"

		//What information do you need for the light ? (position, strength, ...)
		"uniform float u_ambient;\n"
		"uniform float u_diffuse;\n"
		"uniform float u_specular;\n"
		"uniform float u_emissive; \n"
		"uniform vec3 u_light_pos;\n"
		"uniform vec3 lightColour;\n"
		//What information do you need about the object ?
		"uniform vec3 materialColour; \n"
		"uniform float u_shininess;\n"

		"void main() { \n"
		//Compute each of the component needed (specular light, diffuse light, attenuation,...)
		"vec3 L = normalize(u_light_pos - v_frag_coord);\n"
		"vec3 N = v_normal;\n"
		"vec3 R = (reflect(-L,N));\n"
		"vec3 V = normalize(u_view_pos - v_frag_coord);\n"

		"vec3 ambient = u_ambient*materialColour;\n"
		"float diff = max(dot(N,L), 0.0);\n"
		"vec3 diffuse = diff*lightColour;\n"
		"float spec = pow(max(dot(R,V),0.0),u_shininess);\n"
		"vec3 specular = u_specular*spec*lightColour;\n"

		"float attenuation = 1.0/pow(length(u_light_pos - v_frag_coord),2);\n"

		//Compute the value for the light 
		"FragColor = vec4(materialColour*(ambient + u_emissive + (diffuse + specular)*attenuation), 1.0); \n"
		"} \n";

	Shader shader(sourceV, sourceF);


	double prev = 0;
	int deltaFrame = 0;
	//fps function
	auto fps = [&](double now) {
		double deltaTime = now - prev;
		deltaFrame++;
		if (deltaTime > 0.5) {
			prev = now;
			const double fpsCount = (double)deltaFrame / deltaTime;
			deltaFrame = 0;
			std::cout << "\r FPS: " << fpsCount;
			std::cout.flush();
		}
	};

	//Bullet Object
	btRigidBody* sphere = addSphere(1.0, 0.0, 20.0, 0.0, 1.0);
	btTransform t;
	sphere->getMotionState()->getWorldTransform(t);
	btVector3 sphere_translation = t.getOrigin();

	//OpenGL Sphere

	//Path and properties definition
	char sphere_path[] = PATH_TO_OBJECTS "/sphere_smooth.obj";
	Object sphere_render(sphere_path, 1.0, 0.8, 32.0, 0.0);
	sphere_render.makeObject(shader, false);
	glm::vec3 sphere_materialColour = glm::vec3(1.0, 0.0, 0.0);

	//Model matrix definition
	glm::mat4 model = glm::mat4(1.0);
	model = glm::translate(model, glm::vec3(sphere_translation[0], sphere_translation[1], sphere_translation[2]));
	model = glm::scale(model, glm::vec3(0.5, 0.5, 0.5));
	glm::mat4 inverse_model = glm::transpose( glm::inverse(model));

	
	//OpenGL Plane

	//Path and properties definition
	char plane_path[] = PATH_TO_OBJECTS "/plane.obj";
	Object plane(plane_path, 1.0, 0.8, 32.0, 0.0);
	plane.makeObject(shader, false);
	glm::vec3 plane_materialColour = glm::vec3(0.9, 0.6, 0.2);

	//Model matrix definition
	glm::mat4 plane_model = glm::mat4(1.0);
	plane_model = glm::translate(plane_model, glm::vec3(0.0, 0.5, 0.0));
	plane_model = glm::scale(plane_model, glm::vec3(10.0, 10.0, 10.0));
	glm::mat4 inverse_plane_model = glm::transpose(glm::inverse(plane_model));


	//OpenGL Light object

	//Path and properties definition
	Object light(sphere_path, 1.0, 0.8, 10.0, 1.0);
	light.makeObject(shader, false);
	glm::vec3 lightColour = glm::vec3(1.0, 1.0, 1.0);

	//Model matrix definition
	glm::mat4 light_model = glm::mat4(1.0);
	//Definition of a variable for its position since it's gonna move
	glm::vec3 light_pos = glm::vec3(1.0, 2.0, 0.0); 
	light_model = glm::translate(light_model, light_pos);
	light_model = glm::scale(light_model, glm::vec3(0.5, 0.5, 0.5));
	glm::mat4 inverse_light_model = glm::transpose(glm::inverse(light_model));

	
	//Ambient light
	float ambient = 0.3;

	//Camera matrices
	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();

	//Rendering
	glfwSwapInterval(1);
	shader.use();
	shader.setFloat("u_ambient", ambient);

	while (!glfwWindowShouldClose(window)) {
		processInput(window);
		view = camera.GetViewMatrix();

		glfwPollEvents();
		double now = glfwGetTime();

		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Bullet simulation
		world->stepSimulation(1/60.0); //Stepping simulation for one frame

		sphere->getMotionState()->getWorldTransform(t); //Get the position of the bullet object
		sphere_translation = t.getOrigin();				//and put it the sphere_translation vector
		//Changing the model matrix following the sphere_translation
		model = glm::mat4(1.0);							
		model = glm::translate(model, glm::vec3(sphere_translation[0], sphere_translation[1], sphere_translation[2]));
		model = glm::scale(model, glm::vec3(0.5, 0.5, 0.5));
		inverse_model = glm::transpose(glm::inverse(model));

		shader.use();

		//What information do you need to gave to the shader about the light ?
		light_pos = glm::vec3(1*std::cos(now), 2.0, 2 * std::sin(now));
		shader.setVector3f("u_light_pos", light_pos);
		shader.setVector3f("lightColour", lightColour);

		//Camera info
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);
		shader.setVector3f("u_view_pos", camera.Position);

		//Object info
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", sphere_materialColour);
		shader.setFloat("u_diffuse", sphere_render.diffusion_coefficient);
		shader.setFloat("u_specular", sphere_render.specularity_coefficient);
		shader.setFloat("u_shininess", sphere_render.shininess_coefficient);
		shader.setFloat("u_emissive", sphere_render.emission_coefficient);
		sphere_render.draw();

		//Plane info
		shader.setMatrix4("M", plane_model);
		shader.setMatrix4("itM", inverse_plane_model);
		shader.setVector3f("materialColour", plane_materialColour);
		shader.setFloat("u_diffuse", plane.diffusion_coefficient);
		shader.setFloat("u_specular", plane.specularity_coefficient);
		shader.setFloat("u_shininess", plane.shininess_coefficient);
		shader.setFloat("u_emissive", plane.emission_coefficient);
		plane.draw();

		//Light object info
		light_model = glm::translate(glm::mat4(1.0), light_pos);
		light_model = glm::scale(light_model, glm::vec3(0.5, 0.5, 0.5));
		shader.setMatrix4("M", light_model);
		shader.setMatrix4("itM", glm::transpose(glm::inverse(light_model)));
		shader.setVector3f("materialColour", lightColour);
		shader.setFloat("u_diffuse", light.diffusion_coefficient);
		shader.setFloat("u_specular", light.specularity_coefficient);
		shader.setFloat("u_shininess", light.shininess_coefficient);
		shader.setFloat("u_emissive", light.emission_coefficient);
		light.draw();
		
		fps(now);
		glfwSwapBuffers(window);
	}

	//clean up ressource
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	delete broadphase;
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}


void processInput(GLFWwindow* window) {
	
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(LEFT, 0.1);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(RIGHT, 0.1);

	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(FORWARD, 0.1);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboardMovement(BACKWARD, 0.1);

	if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(1, 0.0, 1);
	if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(-1, 0.0, 1);

	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(0.0, 1.0, 1);
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		camera.ProcessKeyboardRotation(0.0, -1.0, 1);


}


