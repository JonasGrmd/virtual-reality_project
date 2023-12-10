#include<iostream>
#include<vector>

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
#include "./../Headers/shaderVFG.h"
#include "./../Headers/object.h"

const int width = 1920;
const int height = 1080;


GLuint compileShader(std::string shaderCode, GLenum shaderType);
GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader);
void processInput(GLFWwindow* window,Shader shader);


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

std::vector<btRigidBody*> bodies_bullet;
std::vector<Object> bodies_render;

//For Bullet object
float shooting_strength = 30.0;

float sphere_radius = 1.0;

float cylinder_diameter = 0.3*2;
float cylinder_height = 2.31;

float box_width = 1.0;
float box_height = 1.0;
float box_depth = 2.0;

//Path and properties definition for OpenGL object
char sphere_path[] = PATH_TO_OBJECTS "/sphere_smooth.obj";
glm::vec3 sphere_materialColour = glm::vec3(1.0, 0.5, 0.5);
glm::vec3 shooted_sphere_materialColour = glm::vec3(0.62, 0.329, 0.98);

char cylinder_path[] = PATH_TO_OBJECTS "/Shooting_cylinder.obj";
glm::vec3 shooted_cylinder_materialColour = glm::vec3(0.973, 1.0, 0.071);

char box_path[] = PATH_TO_OBJECTS "/Shooting_box.obj";
glm::vec3 shooted_box_materialColour = glm::vec3(0.012, 0.902, 0.8);

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

btRigidBody* addCylinder(float d,float h,float x, float y, float z, float mass) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));
	btCylinderShape* cylinder = new btCylinderShape(btVector3(d/2.0,h/2.0,d/2.0));
	btVector3 inertia(btVector3(0.0, 0.0, 0.0));
	if (mass != 0.0) cylinder->calculateLocalInertia(mass, inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, cylinder, inertia);
	btRigidBody* body = new btRigidBody(info);
	world->addRigidBody(body);
	return body;
}

btRigidBody* addBox(float width, float height, float depth, float x, float y, float z, float mass) {
	btTransform t;
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));
	btBoxShape* box = new btBoxShape(btVector3(width / 2.0, height / 2.0, depth / 2.0));
	btVector3 inertia(btVector3(0.0, 0.0, 0.0));
	if (mass != 0.0) box->calculateLocalInertia(mass, inertia);
	btMotionState* motion = new btDefaultMotionState(t);
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, box, inertia);
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

	char fileVert[128] = PATH_TO_SHADERS "/vertexShader.txt";
	char fileFrag[128] = PATH_TO_SHADERS "/fragmentShader.txt";
	Shader shader(fileVert, fileFrag);
	char fileScreenVert[128] = PATH_TO_SHADERS "/screenVertexShader.txt";
	char fileScreenFrag[128] = PATH_TO_SHADERS "/screenFragmentShader.txt";
	Shader screenShader(fileScreenVert, fileScreenFrag);
	char fileDepthVert[128] = PATH_TO_SHADERS "/depthVertexShader.txt";
	char fileDepthFrag[128] = PATH_TO_SHADERS "/depthFragmentShader.txt";
	char fileDepthGeom[128] = PATH_TO_SHADERS "/depthGeometryShader.txt";
	ShaderVFG simpleDepthShader(fileDepthVert, fileDepthFrag, fileDepthGeom);

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

	//Bullet Objects

	int sphere_number = 5;
	for (int i = 0; i < 5; i++) {
		bodies_bullet.push_back(addSphere(sphere_radius, i*0.5, i*5, 0.0, 1.0));
		bodies_bullet.push_back(addCylinder(cylinder_diameter, cylinder_height, i * 0.5, i * 5, 0.0, 1.0));
	}

	//OpenGL Spheres

	for (int i = 0; i < 5; i++) {
		Object sphere_render(sphere_path, 2.0, 1.5, 32.0, 0.0, sphere_materialColour);
		sphere_render.makeObject(shader, false);
		bodies_render.push_back(sphere_render);

		Object cylinder_render(cylinder_path, 2.0, 1.5, 32.0, 0.0, sphere_materialColour);
		cylinder_render.makeObject(shader, false); 
		bodies_render.push_back(cylinder_render); 

	}

	//OpenGL Plane
	//Path and properties definition
	char plane_path[] = PATH_TO_OBJECTS "/plane.obj";
	glm::vec3 plane_materialColour = glm::vec3(0.922, 0.765, 0.349);
	Object plane(plane_path, 1.0, 0.0, 32.0, 0.0, plane_materialColour);
	plane.makeObject(shader, false);
	

	//Model matrix definition
	glm::mat4 plane_model = glm::mat4(1.0);
	plane_model = glm::translate(plane_model, glm::vec3(0.0, 0.0, 0.0));
	plane_model = glm::scale(plane_model, glm::vec3(100.0, 100.0, 100.0));
	glm::mat4 inverse_plane_model = glm::transpose(glm::inverse(plane_model));


	//OpenGL Light object

	//Path and properties definition
	glm::vec3 lightColour = glm::vec3(1.0, 1.0, 1.0);
	Object light(sphere_path, 1.0, 0.8, 32.0, 1.0, lightColour);
	light.makeObject(shader, false);
	

	//Model matrix definition
	glm::mat4 light_model = glm::mat4(1.0);
	//Definition of a variable for its position since it's gonna move
	glm::vec3 light_pos = glm::vec3(1.0, 1.0, 0.0); 
	light_model = glm::translate(light_model, light_pos);
	light_model = glm::scale(light_model, glm::vec3(0.2, 0.2, 0.2));
	glm::mat4 inverse_light_model = glm::transpose(glm::inverse(light_model));


	//Creation of a screen quad

	float quadVertices[] = { // vertex attributes for a quad that fills the entire screen in Normalized Device Coordinates.
		// positions   // texCoords
		-1.0f,  1.0f,  0.0f, 1.0f,
		-1.0f, -1.0f,  0.0f, 0.0f,
		 1.0f, -1.0f,  1.0f, 0.0f,

		-1.0f,  1.0f,  0.0f, 1.0f,
		 1.0f, -1.0f,  1.0f, 0.0f,
		 1.0f,  1.0f,  1.0f, 1.0f
	};
	// screen quad VAO
	unsigned int quadVAO, quadVBO;
	glGenVertexArrays(1, &quadVAO);
	glGenBuffers(1, &quadVBO);
	glBindVertexArray(quadVAO);
	glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), &quadVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));

	
	//Ambient light
	float ambient = 0.3;

	//Camera matrices
	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();

	//Rendering
	glfwSwapInterval(1);
	shader.use();
	shader.setFloat("u_ambient", ambient);
	shader.setInteger("depthMap", 0);

	screenShader.use();
	screenShader.setInteger("colorTexture", 0);
	screenShader.setInteger("normalTexture", 1);

	// framebuffer configuration
	// -------------------------
	unsigned int framebuffer;
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
	// create a color and normal attachment texture
	unsigned int textureColorbuffer;
	unsigned int textureNormalbuffer;
	//generate and set parameters of the color texture
	glGenTextures(1, &textureColorbuffer);
	glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);
	//generate and set parameters of the normal texture
	glGenTextures(1, &textureNormalbuffer);
	glBindTexture(GL_TEXTURE_2D, textureNormalbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, textureNormalbuffer, 0);
	// create a renderbuffer object for depth and stencil attachment (we won't be sampling these)
	unsigned int rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, width, height); // use a single renderbuffer object for both a depth AND stencil buffer.
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo); // now actually attach it
	// tell openGL that we want two textures as the output of this framebuffer
	GLenum attachments[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1 };
	glDrawBuffers(2, attachments);
	// now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
	//we set OpenGL state back to the default onscreen framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	// configure depth map FBO
   // -----------------------
	const unsigned int SHADOW_WIDTH = width, SHADOW_HEIGHT = height;
	unsigned int depthMapFBO;
	glGenFramebuffers(1, &depthMapFBO);
	// create depth cubemap texture
	unsigned int depthCubemap;
	glGenTextures(1, &depthCubemap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
	for (unsigned int i = 0; i < 6; ++i)
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	// attach depth texture as FBO's depth buffer
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthCubemap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	//Initializing the mouse cursor at the center of the screen
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	//Declaring mouse cursor positions
	double xposIn;
	double yposIn;

	while (!glfwWindowShouldClose(window)) {
		processInput(window,shader);
		glfwGetCursorPos(window, &xposIn, &yposIn); 
		camera.ProcessMouseMovement(xposIn, yposIn); 
		view = camera.GetViewMatrix();
		double now = glfwGetTime();

		//Bullet simulation
		world->stepSimulation(1 / 60.0); //Stepping simulation for one frame

		//Light movement
		light_pos = glm::vec3(3*std::cos(now), 1.0, 3*std::sin(now)); 

		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// 0. create depth cubemap transformation matrices
		// -----------------------------------------------
		float near_plane = 1.0f;
		float far_plane = 25.0f;
		glm::mat4 shadowProj = glm::perspective(glm::radians(90.0f), (float)SHADOW_WIDTH / (float)SHADOW_HEIGHT, near_plane, far_plane);
		std::vector<glm::mat4> shadowTransforms;
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));

		// RENDER SCENE TO DEPTH CUBEMAP
		// ----------Initialising cubemap and depth buffer
		glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
		glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
		glClear(GL_DEPTH_BUFFER_BIT);
		simpleDepthShader.use();
		simpleDepthShader.setMatrix4("shadowMatrices[0]", shadowTransforms[0]);
		simpleDepthShader.setMatrix4("shadowMatrices[1]", shadowTransforms[1]);
		simpleDepthShader.setMatrix4("shadowMatrices[2]", shadowTransforms[2]);
		simpleDepthShader.setMatrix4("shadowMatrices[3]", shadowTransforms[3]);
		simpleDepthShader.setMatrix4("shadowMatrices[4]", shadowTransforms[4]);
		simpleDepthShader.setMatrix4("shadowMatrices[5]", shadowTransforms[5]);
		
		simpleDepthShader.setFloat("far_plane", far_plane);
		simpleDepthShader.setVector3f("lightPos", light_pos);
		//-----------Rendering in depth cubemap
		//Objects drawing
		for (int i = 0; i < bodies_bullet.size(); i++) {
			bodies_render[i].draw_on_bullet_object_VFG(simpleDepthShader, bodies_bullet[i], glm::vec3(1.0));
		}
		//Plane drawing
		plane.draw_without_bullet_object_VFG(simpleDepthShader, plane_model);
		//Light drawing
		light_model = glm::translate(glm::mat4(1.0), light_pos);
		light_model = glm::scale(light_model, glm::vec3(0.5, 0.5, 0.5));
		light.draw_without_bullet_object_VFG(simpleDepthShader, light_model);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);

		// bind to framebuffer and draw scene as we normally would to color texture 
		glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
		glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)
		// make sure we clear the framebuffer's content
		glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
		shader.use();
		shader.setFloat("far_plane", far_plane);
		shader.setVector3f("u_light_pos", light_pos);
		shader.setVector3f("lightColour", lightColour);
		//Camera info sent to shader
		shader.setMatrix4("V", view);
		shader.setMatrix4("P", perspective);
		shader.setVector3f("u_view_pos", camera.Position);
		//Objects drawing
		for (int i = 0; i < bodies_bullet.size(); i++) {
			bodies_render[i].draw_on_bullet_object(shader, bodies_bullet[i], glm::vec3(1.0));
		}
		//Plane drawing
		plane.draw_without_bullet_object(shader, plane_model);
		//Light drawing
		light_model = glm::translate(glm::mat4(1.0), light_pos);
		light_model = glm::scale(light_model, glm::vec3(0.5, 0.5, 0.5));
		light.draw_without_bullet_object(shader, light_model);
		

		// now bind back to default framebuffer and draw a quad plane with the attached framebuffer color texture
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
		// clear all relevant buffers
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // set clear color to white (not really necessary actually, since we won't be able to see behind the quad anyways)
		glClear(GL_COLOR_BUFFER_BIT);

		screenShader.use();
		glBindVertexArray(quadVAO);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureColorbuffer);	// use the color attachment texture as the texture of the quad plane
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, textureNormalbuffer);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glActiveTexture(GL_TEXTURE0);
		
		fps(now);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	//clean up ressource
	for (int i = 0;i < bodies_bullet.size();i++) {
		world->removeCollisionObject(bodies_bullet[i]);
		btMotionState* motionState = bodies_bullet[i]->getMotionState();
		btCollisionShape* shape = bodies_bullet[i]->getCollisionShape();
		delete bodies_bullet[i];
		delete shape;
		delete motionState;
	};
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	delete world;
	delete broadphase;
	glDeleteVertexArrays(1, &quadVAO);
	glDeleteBuffers(1, &quadVBO);
	glDeleteRenderbuffers(1, &rbo);
	glDeleteFramebuffers(1, &framebuffer);
	glDeleteFramebuffers(1, &depthMapFBO);
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}

// Random Number generator :

using namespace std;

int randomInt() { return rand(); }

int randomInt(int a, int b)
	{
		if (a > b)
			return randomInt(b, a);
		if (a == b)
			return a;
		return a + (rand() % (b - a));
	}

float randomFloat()
	{
		return (float)(rand()) / (float)(RAND_MAX);
	}

float randomFloat(int a, int b)
	{
		if (a > b)
			return randomFloat(b, a);
		if (a == b)
			return a;

		return (float)randomInt(a, b) + randomFloat();
	}


void processInput(GLFWwindow* window,Shader shader) {
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

	//if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
		//camera.ProcessKeyboardRotation(1, 0.0, 1);
	//if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
		//camera.ProcessKeyboardRotation(-1, 0.0, 1);

	//if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		//camera.ProcessKeyboardRotation(0.0, 1.0, 1);
	//if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		//camera.ProcessKeyboardRotation(0.0, -1.0, 1);

	//Adding sphere to the world
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
		bodies_bullet.push_back(addSphere(sphere_radius, randomFloat(1, 3), 20.0, randomFloat(1, 3), 1.0));
		Object sphere_render(sphere_path, 1.0, 0.8, 32.0, 0.0, sphere_materialColour);
		sphere_render.makeObject(shader, false);
		bodies_render.push_back(sphere_render);

	};

	//Shooting sphere in the world
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {

		//Bullet Object
		btRigidBody* shooting_sphere = addSphere(sphere_radius, camera.Position.x + camera.Front.x * 5.0, camera.Position.y + camera.Front.y * 5.0, camera.Position.z + camera.Front.z * 5.0, 1.0);
		bodies_bullet.push_back(shooting_sphere);
		glm::vec3 shooting_direction = glm::vec3(camera.Front.x * shooting_strength, camera.Front.y * shooting_strength, camera.Front.z * shooting_strength);
		shooting_sphere->setLinearVelocity(btVector3(shooting_direction.x, shooting_direction.y, shooting_direction.z));

		//OpenGL object
		Object sphere_render(sphere_path, 1.0, 0.8, 32.0, 0.0, shooted_sphere_materialColour);
		sphere_render.makeObject(shader, false);
		bodies_render.push_back(sphere_render);
	};

	//Shooting cylinder in the world
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {

		//Bullet Object
		btRigidBody* shooting_cylinder = addCylinder(cylinder_diameter, cylinder_height, camera.Position.x + camera.Front.x * 5.0, camera.Position.y + camera.Front.y * 5.0, camera.Position.z + camera.Front.z * 5.0, 1.0);
		bodies_bullet.push_back(shooting_cylinder);
		glm::vec3 shooting_direction = glm::vec3(camera.Front.x * shooting_strength, camera.Front.y * shooting_strength, camera.Front.z * shooting_strength);
		shooting_cylinder->setLinearVelocity(btVector3(shooting_direction.x, shooting_direction.y, shooting_direction.z));

		//OpenGL object
		Object cylinder_render(cylinder_path, 2.0, 1.5, 32.0, 0.0, shooted_cylinder_materialColour);
		cylinder_render.makeObject(shader, false);
		bodies_render.push_back(cylinder_render);
	};

	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) {

		//Bullet Object
		btRigidBody* shooting_box = addBox(box_width, box_height, box_depth, camera.Position.x + camera.Front.x * 5.0, camera.Position.y + camera.Front.y * 5.0, camera.Position.z + camera.Front.z * 5.0, 1.0);
		bodies_bullet.push_back(shooting_box);
		glm::vec3 shooting_direction = glm::vec3(camera.Front.x * shooting_strength, camera.Front.y * shooting_strength, camera.Front.z * shooting_strength);
		shooting_box->setLinearVelocity(btVector3(shooting_direction.x, shooting_direction.y, shooting_direction.z));

		//OpenGL object
		Object box_render(box_path, 2.0, 1.5, 32.0, 0.0, shooted_box_materialColour);
		box_render.makeObject(shader, false);
		bodies_render.push_back(box_render);
	};
};



