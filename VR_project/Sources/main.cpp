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
#include <chrono> 
#include <thread> 

#include "./../Headers/camera.h"
#include "./../Headers/shader.h"
#include "./../Headers/shaderVFG.h"
#include "./../Headers/object.h"
#include "./../Headers/particle.h"

const int width = 1920;
const int height = 1080;


GLuint compileShader(std::string shaderCode, GLenum shaderType);
GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader);

void processInput(GLFWwindow* window,Shader shader, ShaderVFG simpleDepthShader, btRaycastVehicle* vehicle);

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

bool blur_effect = 1.0;

std::vector<btRigidBody*> bodies_bullet;
std::vector<btRigidBody*> bodies_shaderNL_bullet;
std::vector<Object> bodies_render;
std::vector<Object> bodies_shaderNL_render;

//For Bullet object
float shooting_strength = 100.0;

float sphere_radius = 1.0;

float cylinder_diameter = 0.3*2;
float cylinder_height = 2.31;

float box_width = 1.0;
float box_height = 1.0;
float box_depth = 2.0;

float carMass = 500.0f;

//Path and properties definition for OpenGL object
char sphere_path[] = PATH_TO_OBJECTS "/sphere_smooth.obj";
glm::vec3 sphere_materialColour = glm::vec3(1.0, 0.5, 0.5);
glm::vec3 shooted_sphere_materialColour = glm::vec3(1.0, 0.0, 0.0);

char cylinder_path[] = PATH_TO_OBJECTS "/Shooting_cylinder.obj";
glm::vec3 shooted_cylinder_materialColour = glm::vec3(0.973, 1.0, 0.071);

char box_path[] = PATH_TO_OBJECTS "/Shooting_box.obj";
glm::vec3 shooted_box_materialColour = glm::vec3(0.012, 0.902, 0.8);

char box_texture_path[] = PATH_TO_OBJECTS "/box_texture.obj";

char wheels_r06_h05_path[] = PATH_TO_OBJECTS "/wheels_r06_h05.obj";
glm::vec3 wheels_materialColour = glm::vec3(0.6, 0.6, 0.6);

char canon_path[] = PATH_TO_OBJECTS "/Shooting_cylinder.obj";
glm::vec3 canon_materialColour = glm::vec3(0.4, 0.4, 0.4);

glm::vec3 player_materialColour = glm::vec3(0.0, 0.0, 1.0);

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
	body->setFriction(0.5);
	body->setRollingFriction(.5);
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
	body->setFriction(0.5); 
	body->setRollingFriction(.2);
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
	body->setFriction(0.8); 
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
	world->setGravity(btVector3(0.0, -9.81, 0.0)); 

};
//Functions needed to create a btTriangleMesh from .obj file with the help of ChatGPT
//------------------------------------------------------------------------------------------------------------------------------------
struct MeshData {
	std::vector<btVector3> vertices;
	std::vector<int> indices;  // indices of vertices
};

// Function that take the data extracted and fill it in btTriangleMesh
btTriangleMesh* fillTriangleMesh(const MeshData& meshData) {
	btTriangleMesh* triangleMesh = new btTriangleMesh();

	// Adding each triangle to the mesh
	for (size_t i = 0; i < meshData.indices.size(); i += 3) {
		// Taking the indices of vertices
		int index0 = meshData.indices[i];
		int index1 = meshData.indices[i + 1];
		int index2 = meshData.indices[i + 2];

		// Checking if indices are valid
		if (index0 < 0 || index0 >= static_cast<int>(meshData.vertices.size()) ||
			index1 < 0 || index1 >= static_cast<int>(meshData.vertices.size()) ||
			index2 < 0 || index2 >= static_cast<int>(meshData.vertices.size())) {
			std::cout << "Bugged index: " << index0 << std::endl;
			std::cout << "Bugged index: " << index1 << std::endl;
			std::cout << "Bugged index: " << index2 << std::endl;
			delete triangleMesh;
			return nullptr;
		}

		// Taking the vertices of traingle
		btVector3 vertex0 = meshData.vertices[index0];
		btVector3 vertex1 = meshData.vertices[index1];
		btVector3 vertex2 = meshData.vertices[index2];

		// Adding the triangle to the mesh
		triangleMesh->addTriangle(vertex0, vertex1, vertex2);
	}

	return triangleMesh;
}

// Function that load the mesh from .obj file
MeshData loadMeshFromObj(const char* filePath) {
	MeshData meshData;

	std::ifstream file(filePath);
	if (!file.is_open()) {
		return meshData;
	}

	std::string line;
	while (std::getline(file, line)) {
		std::istringstream iss(line);
		std::string token;
		iss >> token;

		if (token == "v") {
			// reading the vertices coordinates of each triangle
			btVector3 vertex;
			iss >> vertex[0] >> vertex[1] >> vertex[2];
			meshData.vertices.push_back(vertex);
		}
		else if (token == "f") {
			std::string f1, f2, f3;
			iss >> f1 >> f2 >> f3;

			std::string p, t, n;
			p = f1.substr(0, f1.find("/"));
			f1.erase(0, f1.find("/") + 1);

			t = f1.substr(0, f1.find("/"));
			f1.erase(0, f1.find("/") + 1);

			n = f1.substr(0, f1.find("/"));

			meshData.indices.push_back(std::stoi(p) - 1);

			p = f2.substr(0, f2.find("/"));
			f2.erase(0, f2.find("/") + 1);

			t = f2.substr(0, f2.find("/"));
			f2.erase(0, f2.find("/") + 1);

			n = f2.substr(0, f2.find("/"));

			meshData.indices.push_back(std::stoi(p) - 1);

			p = f3.substr(0, f3.find("/"));
			f3.erase(0, f3.find("/") + 1);

			t = f3.substr(0, f3.find("/"));
			f3.erase(0, f3.find("/") + 1);

			n = f3.substr(0, f3.find("/"));

			meshData.indices.push_back(std::stoi(p) - 1);
			// Debugging exit
			std::cout << "Vertex Index: " << std::stoi(p) - 1 << std::endl;
		};
	}
	return meshData;
}
//-------------------------------------------------------------------------------------------------------------------------------------------

//Loading Cubemap Function (copied form LearnOpenGL)
unsigned int loadCubemap(std::vector<std::string> faces)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

	int width, height, nrChannels;
	for (unsigned int i = 0; i < faces.size(); i++)
	{
		unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &nrChannels, 0);
		if (data)
		{
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
				0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data
			);
			stbi_image_free(data);
		}
		else
		{
			std::cout << "Cubemap tex failed to load at path: " << faces[i] << std::endl;
			stbi_image_free(data);
		}
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	return textureID;
}

int main(int argc, char* argv[])
{
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

	char fileVert0[128] = PATH_TO_SHADERS "/normalLightningVertexShader.txt";
	char fileFrag0[128] = PATH_TO_SHADERS "/normalLightningFragmentShader.txt";
	Shader shaderNL(fileVert0, fileFrag0);
	char fileVert[128] = PATH_TO_SHADERS "/cartoonLightningVertexShader.txt";
	char fileFrag[128] = PATH_TO_SHADERS "/cartoonLightningFragmentShader.txt";
	Shader shader(fileVert, fileFrag);
	char fileScreenVert[128] = PATH_TO_SHADERS "/screenVertexShader.txt";
	char fileScreenFrag[128] = PATH_TO_SHADERS "/screenFragmentShader.txt";
	Shader screenShader(fileScreenVert, fileScreenFrag);
	char fileDepthVert[128] = PATH_TO_SHADERS "/depthVertexShader.txt";
	char fileDepthFrag[128] = PATH_TO_SHADERS "/depthFragmentShader.txt";
	char fileDepthGeom[128] = PATH_TO_SHADERS "/depthGeometryShader.txt";
	ShaderVFG simpleDepthShader(fileDepthVert, fileDepthFrag, fileDepthGeom);
	char fileParticleVert[128] = PATH_TO_SHADERS "/particleVertexShader.txt";
	char fileParticleFrag[128] = PATH_TO_SHADERS "/particleFragmentShader.txt";
	Shader particleShader(fileParticleVert, fileParticleFrag);
	char fileCubeMapVert[128] = PATH_TO_SHADERS "/cubeMapVertexShader.txt";
	char fileCubeMapFrag[128] = PATH_TO_SHADERS "/cubeMapFragmentShader.txt";
	Shader skyBoxShader(fileCubeMapVert, fileCubeMapFrag);
	char fileReflectiveVert[128] = PATH_TO_SHADERS "/reflectiveObjectVertexShader.txt";
	char fileReflectiveFrag[128] = PATH_TO_SHADERS "/reflectiveObjectFragmentShader.txt";
	Shader reflectiveObjectShader(fileReflectiveVert, fileReflectiveFrag);


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
	//Initialize Bullet world
	init();

	//Bullet and OpenGL ground
	//---------------------------------------------------------------------
	char ground_path[] = PATH_TO_OBJECTS "/Ground2.obj";
	//Loading the mesh
	MeshData meshData = loadMeshFromObj(ground_path);
	// Creating a bullet collision shape from the loaded mesh
	btBvhTriangleMeshShape* collisionShapeGround = new btBvhTriangleMeshShape(fillTriangleMesh(meshData), true, true);
	// Position of the ground
	btTransform t; 
	t.setIdentity(); 
	t.setOrigin(btVector3(0.0, 0.0, 0.0)); 
	btMotionState* motionStateGround = new btDefaultMotionState(t);
	// Construction of the RigidBody
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0.0, motionStateGround, collisionShapeGround);
	btRigidBody* rigidBodyGround = new btRigidBody(rigidBodyCI); 
	rigidBodyGround->setFriction(0.5);
	world->addRigidBody(rigidBodyGround);

	// OpenGL object for the ground 
	glm::vec3 ground_materialColour = glm::vec3(0.922, 0.765, 0.349);
	Object ground(ground_path, 1.0, 0.0, 32.0, 0.0, ground_materialColour);
	ground.makeObject(shader, simpleDepthShader, false);

	//Model matrix definition
	glm::mat4 ground_model = glm::mat4(1.0);
	ground_model = glm::translate(ground_model, glm::vec3(0.0, 0.0, 0.0));
	ground_model = glm::scale(ground_model, glm::vec3(1.0));
	glm::mat4 inverse_ground_model = glm::transpose(glm::inverse(ground_model)); 
	//---------------------------------------------------------------------
	//Bullet and OpenGL sphere and cylinder
	//---------------------------------------------------------------------
	int sphere_number = 6;
	float pi = 3.14159265359;
	for (int i = 0; i < sphere_number; i++) {
		if (i < sphere_number / 2) {
			bodies_shaderNL_bullet.push_back(addSphere(sphere_radius, 8 * cos((i + 1) * pi / 3), 5.0, 8 * sin((i + 1) * pi / 3), 1.0));
		}
		else
		{
			bodies_bullet.push_back(addSphere(sphere_radius, 8 * cos((i + 1) * pi / 3), 5.0, 8 * sin((i + 1) * pi / 3), 1.0));
		}
	}

	for (int i = 0; i < sphere_number; i++) {
		Object sphere_render(sphere_path, 1.0, 1.0, 32.0, 0.0, sphere_materialColour);
		if (i < sphere_number/2) {
			sphere_render.makeObject(shaderNL, simpleDepthShader, false);
			bodies_shaderNL_render.push_back(sphere_render); 
		}
		else
		{
			sphere_render.makeObject(shader, simpleDepthShader, false);
			bodies_render.push_back(sphere_render);
		}
	}
	//------------------------------------------------------------------------
	//Bullet and OpenGL columns of cubes
	//--------------------------------------------------------------------------------------------------
	char cube_path[] = PATH_TO_OBJECTS "/cube.obj";
	glm::vec3 cube_materialColour = glm::vec3(0.85, 0.0, 0.85); 
	float cube_mass = 0.8;
	float y_it = 0.0;

	for (int i = 0;i < 10;i++) {
		y_it += 2.01;
		//column 1 (60 cubes)
		bodies_bullet.push_back(addBox( 2*1.0, 2*1.0, 2*1.0, -25.0, 0.5+y_it, -30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -25.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, -30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -29.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, -34.0, cube_mass));

		//column 2 (60 cubes)
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 25.0, 0.5 + y_it, 30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 25.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, 30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 29.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, 34.0, cube_mass));

		//column 3 (60 cubes)
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 25.0, 0.5 + y_it, -30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 25.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, -30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 29.0, 0.5 + y_it, -32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, 27.0, 0.5 + y_it, -34.0, cube_mass));

		//column 4 (60 cubes)
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -25.0, 0.5 + y_it, 30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -25.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, 30.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -29.0, 0.5 + y_it, 32.0, cube_mass));
		bodies_bullet.push_back(addBox(2 * 1.0, 2 * 1.0, 2 * 1.0, -27.0, 0.5 + y_it, 34.0, cube_mass));
	}
	for (int i = 0;i < 240;i++) {
		Object cube_render(cube_path, 1.0, 1.0, 32.0, 0.0, cube_materialColour);
		cube_render.makeObject(shader, simpleDepthShader, false);
		bodies_render.push_back(cube_render);
	}
	//---------------------------------------------------------------------------------------------
	
	//Bullet and OpenGl car (due to a lack of documentation about Bullet Physics the following bullet creation has been made with the help of ChatGPT)
	//----------------------------------------------------------------------
	//Creating the main core of the car (box)
	btCollisionShape* carShape = new btBoxShape(btVector3(box_width, 1.2, box_depth)); 

	//Center of mass
	btVector3 localInertia(0, 0, 0); 
	carShape->calculateLocalInertia(carMass, localInertia);

	// Initial position of the car in the world
	btVector3 initialPosition(0.0, 20.0, 0.0);
	btTransform transf;
	transf.setIdentity();
	transf.setOrigin(initialPosition);
	btMotionState* carMotionState = new btDefaultMotionState(transf);

	btRigidBody::btRigidBodyConstructionInfo carRigidBodyCI(carMass, carMotionState, carShape, localInertia);  
	btRigidBody* carRigidBody = new btRigidBody(carRigidBodyCI); 

	//Friction of the main core
	carRigidBody->setFriction(0.7);
	carRigidBody->setRollingFriction(0.7);
	carRigidBody->setAngularFactor(btVector3(1, 1, 1)); 

	// Adding tuning parameters
	btRaycastVehicle::btVehicleTuning tuning; 
	tuning.m_suspensionStiffness = 20.0; 
	tuning.m_suspensionDamping = 2.3; 

	btVehicleRaycaster* vehicleRayCaster = new btDefaultVehicleRaycaster(world); 
	btRaycastVehicle* vehicle = new btRaycastVehicle(tuning, carRigidBody, vehicleRayCaster); 

	//Damping of wheels
	btScalar linearDamping(0.2); 
	btScalar angularDamping(0.1);  

	vehicle->getRigidBody()->setDamping(linearDamping, angularDamping); 

	vehicle->getRigidBody()->setMassProps(carMass, btVector3(0, 0, 0)); 

	world->addRigidBody(carRigidBody); 

	// Positions of wheels relative to center of the car
	btVector3 connectionPointCS0_0(-1.5, -1.0, 1.5); //avant gauche
	btVector3 connectionPointCS0_1(1.5, -1.0, 1.5); //avant droit
	btVector3 connectionPointCS0_2(-1.5, -1.0, -1.5); //arrière gauche
	btVector3 connectionPointCS0_3(1.5, -1.0, -1.5); //arrière droit

	//Orientation of wheels
	btVector3 wheelDirectionCS0(0, -1, 0); 
	btVector3 wheelAxleCS(-1, 0, 0); 

	// Adding the wheels to the vehicle
	vehicle->addWheel(connectionPointCS0_0, wheelDirectionCS0, wheelAxleCS, 0.6, 0.5, tuning, true); //cylinder of radius 0.6 and height 0.5
	vehicle->addWheel(connectionPointCS0_1, wheelDirectionCS0, wheelAxleCS, 0.6, 0.5, tuning, true); 
	vehicle->addWheel(connectionPointCS0_2, wheelDirectionCS0, wheelAxleCS, 0.6, 0.5, tuning, true);
	vehicle->addWheel(connectionPointCS0_3, wheelDirectionCS0, wheelAxleCS, 0.6, 0.5, tuning, true);

	// Adding a canon under the form of a wheel
	vehicle->addWheel(btVector3(0.0,1.2 , 0.0), wheelDirectionCS0, wheelAxleCS,0.3,0.3,tuning,true); 

	//Friction of the wheels
	for (int i = 0;i < 4;i++) {
		vehicle->getWheelInfo(i).m_frictionSlip = 10.0;
		vehicle->getWheelInfo(i).m_wheelsDampingRelaxation = 0.8; 
		vehicle->getWheelInfo(i).m_wheelsDampingCompression = 0.8; 
	}

	world->addVehicle(vehicle); 

	//OpenGL object of the car
	Object vehicle_core_render(box_path, 2.0, 1.5, 32.0, 0.0, player_materialColour);  
	vehicle_core_render.makeObject(shader, simpleDepthShader, false);

	Object vehicle_wheel0_render(wheels_r06_h05_path, 2.0, 1.5, 32.0, 0.0, wheels_materialColour);
	vehicle_wheel0_render.makeObject(shader, simpleDepthShader, false);
	Object vehicle_wheel1_render(wheels_r06_h05_path, 2.0, 1.5, 32.0, 0.0, wheels_materialColour);
	vehicle_wheel1_render.makeObject(shader, simpleDepthShader, false);
	Object vehicle_wheel2_render(wheels_r06_h05_path, 2.0, 1.5, 32.0, 0.0, wheels_materialColour);
	vehicle_wheel2_render.makeObject(shader, simpleDepthShader, false);
	Object vehicle_wheel3_render(wheels_r06_h05_path, 2.0, 1.5, 32.0, 0.0, wheels_materialColour);
	vehicle_wheel3_render.makeObject(shader, simpleDepthShader, false);

	Object vehicle_canon_render(canon_path, 2.0, 1.5, 32.0, 0.0, canon_materialColour);
	vehicle_canon_render.makeObject(shader, simpleDepthShader, false); 

	//-----------------------------------------------------------

	//OpenGL Light object
	//-----------------------------------------------------------
	//Path and properties definition
	glm::vec3 lightColour = glm::vec3(1.0, 1.0, 1.0);
	Object light(sphere_path, 1.0, 0.8, 32.0, 1.0, lightColour);
	light.makeObject(shader, simpleDepthShader, false);
	//------------------------------------------------------------

	//OpenGL Reflective object
	//------------------------------------------------------------
	Object reflectiveSphere(sphere_path, 1.0, 0.8, 32.0, 1.0, lightColour);
	reflectiveSphere.makeObject(reflectiveObjectShader, simpleDepthShader, false);
	glm::mat4 reflectiveSphereModel = glm::mat4(1.0);
	reflectiveSphereModel = glm::translate(reflectiveSphereModel, glm::vec3(0.0, 50.0, 0.0));
	reflectiveSphereModel = glm::scale(reflectiveSphereModel, glm::vec3(20.0));


	//SKYBOX CONFIGURATION
	// -----------------------------------------------------------
	//Skybox Vertices
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
	//Skybox VAO Generation
	unsigned int skyboxVAO, skyboxVBO;
	glGenVertexArrays(1, &skyboxVAO);
	glGenBuffers(1, &skyboxVBO);
	glBindVertexArray(skyboxVAO);
	glBindBuffer(GL_ARRAY_BUFFER, skyboxVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(skyboxVertices), &skyboxVertices, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	//Skybox and cubemap loading
	std::vector<std::string> faces
	{
		PATH_TO_TEXTURE "/skybox/right.jpg",
		PATH_TO_TEXTURE "/skybox/left.jpg",
		PATH_TO_TEXTURE "/skybox/top.jpg",
		PATH_TO_TEXTURE "/skybox/bottom.jpg",
		PATH_TO_TEXTURE "/skybox/front.jpg",
		PATH_TO_TEXTURE "/skybox/back.jpg",
		//PATH_TO_TEXTURE "/skybox/px.jpg",
		//PATH_TO_TEXTURE "/skybox/nx.jpg",
		//PATH_TO_TEXTURE "/skybox/py.jpg",
		//PATH_TO_TEXTURE "/skybox/ny.jpg",
		//PATH_TO_TEXTURE "/skybox/pz.jpg",
		//PATH_TO_TEXTURE "/skybox/nz.jpg",
	};
	unsigned int cubemapTexture = loadCubemap(faces);
	//------------------------------------------------------------------


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

	// framebuffer configuration
	// -------------------------
	unsigned int framebuffer;
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
	// create a color and normal attachment texture
	unsigned int textureColorbuffer;
	unsigned int textureNormalbuffer;
	unsigned int textureDepthbuffer;
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
	//generate and set paramétéers of the depth texture
	glGenTextures(1, &textureDepthbuffer);
	glBindTexture(GL_TEXTURE_2D, textureDepthbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, textureDepthbuffer, 0);
	// tell openGL that we want two textures as the output of this framebuffer
	GLenum attachments[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_DEPTH_ATTACHMENT};
	glDrawBuffers(2, attachments);
	// now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
	//we set OpenGL state back to the default onscreen framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	// configure depth map FBO
   // -----------------------
	const unsigned int SHADOW_WIDTH = 1920.0, SHADOW_HEIGHT = 1920.0;
	unsigned int depthMapFBO;
	glGenFramebuffers(1, &depthMapFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
	// create depth cubemap texture
	unsigned int depthCubemap;
	glGenTextures(1, &depthCubemap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
	for (unsigned int i = 0; i < 6; ++i)
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	
	// attach depth texture as FBO's depth buffer
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthCubemap, 0);
	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE) {
		// Gérer l'erreur, par exemple, afficher un message d'erreur
		std::cerr << "Framebuffer is not complete!" << std::endl;
	}
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	//Initializing the mouse cursor at the center of the screen
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); 

	//Declaring mouse cursor positions
	double xposIn; 
	double yposIn;

	//Ambient light
	float ambient = 0.2;

	//Camera matrices
	glm::mat4 view = camera.GetViewMatrix();
	glm::mat4 perspective = camera.GetProjectionMatrix();

	//Particles system
	ParticleGenerator particleGen(particleShader, 100, camera);

	//Rendering
	glfwSwapInterval(1);

	glm::vec3 background_color = glm::vec3(250.0/255,180.0/255, 140.0/255);
	
	screenShader.use();
	screenShader.setInteger("colorTexture", 0);
	screenShader.setInteger("normalTexture", 1);
	screenShader.setInteger("depthTexture", 2);
	screenShader.setVector3f("backgroundColor", background_color);

	shader.use();
	shader.setFloat("u_ambient", ambient);
	shader.setInteger("depthMap", 0);

	shaderNL.use();
	shaderNL.setFloat("u_ambient", ambient);
	shaderNL.setInteger("depthMap", 0);
  
	skyBoxShader.use();
	skyBoxShader.setInteger("skybox", 0);

	reflectiveObjectShader.use();
	reflectiveObjectShader.setInteger("skybox", 0);

	glm::vec3 light_pos; 
	glm::mat4 light_model;
  
  double lastShootTime = 0.0;
	const double shootDelay = 0.5; // delay between the shootings

	while (!glfwWindowShouldClose(window)) {
		//Bullet simulation
		world->clearForces(); 
		world->stepSimulation(1 / 60.0); //Stepping simulation for one frame 
		processInput(window,shader, simpleDepthShader, vehicle);

		// Shooting of spheres in front of the canon
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && (glfwGetTime() - lastShootTime) > shootDelay) {

			lastShootTime = glfwGetTime();
			// Front wheels positions
			btVector3 wheelPos0 = vehicle->getWheelInfo(0).m_worldTransform.getOrigin();
			btVector3 wheelPos1 = vehicle->getWheelInfo(1).m_worldTransform.getOrigin();

			// Average position of front wheels
			btVector3 avgWheelPos = (wheelPos0 + wheelPos1) * 0.5;

			// Initial position of the sphere in front of the canon
			btVector3 baseNormalLocal = vehicle->getWheelInfo(4).m_worldTransform.getBasis() * btVector3(0, 1, 0);
			btVector3 wheelPositionWorld = vehicle->getWheelInfo(4).m_worldTransform * vehicle->getWheelInfo(4).m_chassisConnectionPointCS; 
			btVector3 initialPosition = btVector3(wheelPositionWorld.getX() + baseNormalLocal.getX(), wheelPositionWorld.getY(), wheelPositionWorld.getZ()+ baseNormalLocal.getZ());

			// Creating the bullet shooting spheres object
			btRigidBody* shootingSphere = addSphere(sphere_radius, initialPosition.getX(), initialPosition.getY(), initialPosition.getZ(), 1.0);
			bodies_bullet.push_back(shootingSphere);

			// Applying velocity a the shooted sphere in the direction of the canon
			btVector3 baseNormalLocal_norm = baseNormalLocal.normalize();
			shootingSphere->setLinearVelocity(baseNormalLocal_norm * shooting_strength);

			// OpenGl object for shooted sphere
			Object sphereRender(sphere_path, 1.0, 0.8, 32.0, 0.0, shooted_sphere_materialColour);
			sphereRender.makeObject(shader, simpleDepthShader, false);
			bodies_render.push_back(sphereRender);
		};

		//Camera on vehicle
		glfwGetCursorPos(window, &xposIn, &yposIn);  
		view = camera.GetViewMatrixOnPlayer(xposIn, yposIn, vehicle_core_render.getObjectPosition(carRigidBody),20.0);

		double now = glfwGetTime();


		//Light movement
		light_pos = glm::vec3(5.0*sin(now), 10.0, 5.0*cos(now));
		light_model = glm::translate(glm::mat4(1.0), light_pos);
		light_model = glm::scale(light_model, glm::vec3(0.1, 0.1, 0.1));

		// 0. create depth cubemap transformation matrices
		// -----------------------------------------------
		float near_plane = 1.0f;
		float far_plane = 100.0f;

		glm::mat4 shadowProj = glm::perspective(glm::radians(90.0f), (float)SHADOW_WIDTH / (float)SHADOW_HEIGHT, near_plane, far_plane);
		std::vector<glm::mat4> shadowTransforms;
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
		shadowTransforms.push_back(shadowProj * glm::lookAt(light_pos, light_pos + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));

		// RENDER SCENE TO DEPTH CUBEMAP
		// ---------- Initialising cubemap and depth buffer
		glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
		glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
		glEnable(GL_DEPTH_TEST);
		glClear(GL_DEPTH_BUFFER_BIT);

		simpleDepthShader.use();

		for (unsigned int i = 0; i < 6; ++i) {
			std::string uniformName = "shadowMatrices[" + std::to_string(i) + "]";
			simpleDepthShader.setMatrix4(uniformName.c_str(), shadowTransforms[i]);
		}
		simpleDepthShader.setFloat("far_plane", far_plane);
		simpleDepthShader.setVector3f("light_pos", light_pos);

		//----------- Rendering in depth cubemap
		//Objects drawing
		for (int i = 0; i < bodies_bullet.size(); i++) {
			bodies_render[i].draw_on_bullet_object_VFG(simpleDepthShader, bodies_bullet[i], glm::vec3(1.0));
		}
		for (int i = 0; i < bodies_shaderNL_bullet.size(); i++) {
			bodies_shaderNL_render[i].draw_on_bullet_object_VFG(simpleDepthShader, bodies_shaderNL_bullet[i], glm::vec3(1.0));
		}
		//Ground drawing
		ground.draw_without_bullet_object_VFG(simpleDepthShader, ground_model); 

		//Vehicle drawing
		vehicle_core_render.draw_on_bullet_object_vehicle_core_VFG(simpleDepthShader, vehicle, glm::vec3(1.0));
		vehicle_wheel0_render.draw_on_bullet_object_vehicle_wheels_VFG(simpleDepthShader, vehicle, 0, glm::vec3(1.0));
		vehicle_wheel1_render.draw_on_bullet_object_vehicle_wheels_VFG(simpleDepthShader, vehicle, 1, glm::vec3(1.0));
		vehicle_wheel2_render.draw_on_bullet_object_vehicle_wheels_VFG(simpleDepthShader, vehicle, 2, glm::vec3(1.0));
		vehicle_wheel3_render.draw_on_bullet_object_vehicle_wheels_VFG(simpleDepthShader, vehicle, 3, glm::vec3(1.0));
		vehicle_canon_render.draw_on_bullet_object_vehicle_wheels_VFG(simpleDepthShader, vehicle, 4, glm::vec3(0.5));
		glBindFramebuffer(GL_FRAMEBUFFER, 0);


		//RENDER SCENE TO MAIN FRAMEBUFFER
		// 
		// bind to framebuffer and draw scene as we normally would to color texture 
		glViewport(0, 0, width, height);
		glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
		glEnable(GL_DEPTH_TEST); // enable depth testing (is disabled for rendering screen-space quad)
		// make sure we clear the framebuffer's content
		glClearColor(background_color.x, background_color.y, background_color.z, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);

		shader.use();
		shader.setFloat("far_plane", far_plane);
		shader.setFloat("near_plane", near_plane);
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

		//Vehicle drawing
		vehicle_core_render.draw_on_bullet_object_vehicle_core(shader, vehicle, glm::vec3(1.0));
		vehicle_wheel0_render.draw_on_bullet_object_vehicle_wheels(shader, vehicle,0, glm::vec3(1.0));
		vehicle_wheel1_render.draw_on_bullet_object_vehicle_wheels(shader, vehicle,1, glm::vec3(1.0));
		vehicle_wheel2_render.draw_on_bullet_object_vehicle_wheels(shader, vehicle,2, glm::vec3(1.0));
		vehicle_wheel3_render.draw_on_bullet_object_vehicle_wheels(shader, vehicle,3, glm::vec3(1.0));
		vehicle_canon_render.draw_on_bullet_object_vehicle_wheels(shader, vehicle, 4, glm::vec3(0.5));

		//Ground drawing
		ground.draw_without_bullet_object(shader, ground_model);
		//Light drawing
		light.draw_without_bullet_object(shader, light_model);
    
		shaderNL.use(); 
		shaderNL.setFloat("far_plane", far_plane); 
		shaderNL.setFloat("near_plane", near_plane); 
		shaderNL.setVector3f("u_light_pos", light_pos); 
		shaderNL.setVector3f("lightColour", lightColour); 
		//Camera info sent to shader
		shaderNL.setMatrix4("V", view); 
		shaderNL.setMatrix4("P", perspective); 
		shaderNL.setVector3f("u_view_pos", camera.Position); 
		for (int i = 0; i < bodies_shaderNL_bullet.size(); i++) { 
			bodies_shaderNL_render[i].draw_on_bullet_object(shaderNL, bodies_shaderNL_bullet[i], glm::vec3(1.0)); 
		}

		reflectiveObjectShader.use();
		reflectiveObjectShader.setFloat("far_plane", far_plane);
		reflectiveObjectShader.setMatrix4("V", view);
		reflectiveObjectShader.setMatrix4("P", perspective);
		reflectiveObjectShader.setVector3f("cameraPos", camera.Position);
		reflectiveSphere.draw_with_reflective_texture(reflectiveObjectShader, reflectiveSphereModel, cubemapTexture);
		
		//Draw skybox as last
		glDepthFunc(GL_LEQUAL);  // change depth function so depth test passes when values are equal to depth buffer's content
		skyBoxShader.use();
		glm::mat4 view_for_cubemap = glm::mat4(glm::mat3(view));
		skyBoxShader.setMatrix4("V", view_for_cubemap);
		skyBoxShader.setMatrix4("P", perspective);
		// skybox cube
		glBindVertexArray(skyboxVAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubemapTexture);
		glDrawArrays(GL_TRIANGLES, 0, 36);
		glBindVertexArray(0);
		glDepthFunc(GL_LESS); // set depth function back to default


		//PUT RENDERED SCENE TEXTURE ON SCREENBUFFER
		// 
		// now bind back to default framebuffer and draw a quad plane with the attached framebuffer color texture
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glDisable(GL_DEPTH_TEST); // disable depth test so screen-space quad isn't discarded due to depth test.
		// clear all relevant buffers
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // set clear color to white (not really necessary actually, since we won't be able to see behind the quad anyways)
		glClear(GL_COLOR_BUFFER_BIT);

		screenShader.use();
		screenShader.setBool("blur", blur_effect);

		glBindVertexArray(quadVAO);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, textureColorbuffer);	// use the color attachment texture as the texture of the quad plane
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, textureNormalbuffer);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, textureDepthbuffer);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		
		fps(now);
		glfwSwapBuffers(window);
		glfwPollEvents();

	}
	
	// Nettoyer les ressources Bullet Physics
	for (int i = 0; i < bodies_bullet.size(); i++) { 
		btCollisionObject* obj = bodies_bullet[i]; 
		world->removeCollisionObject(obj); 
		btCollisionShape* shape = obj->getCollisionShape(); 
		delete obj;  // Ne supprime que le pointeur, pas l'objet réel
		delete shape; 
	}
	for (int i = 0; i < bodies_shaderNL_bullet.size(); i++) {
		btCollisionObject* obj = bodies_shaderNL_bullet[i];
		world->removeCollisionObject(obj);
		btCollisionShape* shape = obj->getCollisionShape();
		delete obj;  // Ne supprime que le pointeur, pas l'objet réel
		delete shape;
	}

	// We free the allocated memory
	// Deleting Bullet vehicle
	delete vehicleRayCaster; 
	delete vehicle; 

	// Deleting Bullet rigid bodies
	delete rigidBodyGround; 
	delete carRigidBody;

	// Deleting Bullet motion objects and collision shapes
	delete collisionShapeGround; 
	delete motionStateGround; 
	delete carMotionState; 
	delete carShape;

	// Deleting Bullet last components
	delete world; 
	delete broadphase; 
	delete solver; 
	delete collisionConfig; 
	delete dispatcher; 

	// Cleaning OpenGL ressources
	glDeleteVertexArrays(1, &quadVAO); 
	glDeleteBuffers(1, &quadVBO); 
	glDeleteFramebuffers(1, &framebuffer); 
	glDeleteFramebuffers(1, &depthMapFBO); 
	glDeleteVertexArrays(1, &skyboxVAO);
	glDeleteBuffers(1, &skyboxVBO);

	// Closing GLFW window
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

// Tout ce qui concerne le mouvement du véhicule dans la suite du code a été fait à l'aide de ChatGPT.
// Paramètres de mouvement du véhicule :

const float accelerationForce = 10000.0f;  // Force d'accélération
float brakeForce = -10000.0f;         // Force de freinage
const float steeringIncrement = 0.05f; 
const float steeringIncrement2 = 0.03f;// Incrément de rotation
const float decelerationForce = -100.0f;  // Force de décélération
float maxSteeringAngle = 1.6;
bool B_pressed = 0.0;


void processInput(GLFWwindow* window, Shader shader, ShaderVFG simpleDepthShader, btRaycastVehicle* vehicle) {
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	//Moving the vehicle

	// Acceleration
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
		vehicle->applyEngineForce(accelerationForce, 2);
		vehicle->applyEngineForce(accelerationForce, 3);
	}
	else {
		vehicle->applyEngineForce(0.0, 2);
		vehicle->applyEngineForce(0.0, 3);
	}


	//Adding sphere to the world
	if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
		bodies_bullet.push_back(addSphere(sphere_radius, randomFloat(1, 3), 20.0, randomFloat(1, 3), 1.0));
		Object sphere_render(sphere_path, 1.0, 0.8, 32.0, 0.0, sphere_materialColour);
		sphere_render.makeObject(shader, simpleDepthShader, false); 
		bodies_render.push_back(sphere_render);
	}
	
	// Brake
	if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
		vehicle->applyEngineForce(brakeForce, 2); 
		vehicle->applyEngineForce(brakeForce, 3); 
	}
	// Left rotation
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
		float currentSteering = vehicle->getSteeringValue(0); 
		float newSteering = currentSteering + steeringIncrement;

		vehicle->setSteeringValue(newSteering, 0); 
		vehicle->setSteeringValue(newSteering, 1);
		vehicle->setSteeringValue(newSteering, 2);
		vehicle->setSteeringValue(newSteering, 3);
	}
	// Right rotation
	else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
		float currentSteering = vehicle->getSteeringValue(0);
		float newSteering = currentSteering - steeringIncrement;

		vehicle->setSteeringValue(newSteering, 0);
		vehicle->setSteeringValue(newSteering, 1);
		vehicle->setSteeringValue(newSteering, 2); 
		vehicle->setSteeringValue(newSteering, 3); 
	}
	// Canon left rotation
	if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
		float currentSteering = vehicle->getSteeringValue(4);
		float newSteering = currentSteering + steeringIncrement2;

		vehicle->setSteeringValue(newSteering, 4);
	}
	// Canon right rotation
	else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
		float currentSteering = vehicle->getSteeringValue(4);
		float newSteering = currentSteering - steeringIncrement2;


		vehicle->setSteeringValue(newSteering, 4);
	}

	if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS && B_pressed != 1.0) {
		blur_effect = not blur_effect;
		std::cout << blur_effect << endl;
		B_pressed = 1.0;
	};

	if (glfwGetKey(window, GLFW_KEY_B) != GLFW_PRESS && B_pressed == 1.0) {
		B_pressed = 0.0;
	};
};



