#ifndef OBJECT_H
#define OBJECT_H

#include<iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

#include <src/btBulletCollisionCommon.h>
#include <src/btBulletDynamicsCommon.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include "./../Headers/shader.h"


/*Principe :
* On donne le path du fichier -> on lit le fichier
* 2 ?tape
* 1)load le model -> lit le fichier ligne par ligne
* liste de position de normal de texture
* suivant la premi?re lettre : lit les valeur suivant et les met dans un vec puis push dans la bonne liste
* en gros sotck les data dans une frome de tableau
*/

struct Vertex {
	glm::vec3 Position;
	glm::vec2 Texture;
	glm::vec3 Normal;
};


class Object
{
public:
	std::vector<glm::vec3> positions;
	std::vector<glm::vec2> textures;
	std::vector<glm::vec3> normals;
	std::vector<Vertex> vertices;

	int numVertices;

	float diffusion_coefficient;
	float specularity_coefficient;
	float shininess_coefficient;
	float emission_coefficient;
	glm::vec3 materialColour;

	GLuint VBO, VAO;

	glm::mat4 model = glm::mat4(1.0);


	Object(const char* path, float diffusion, float specularity, float shininess, float emmision, glm::vec3 materialColour) {

		//Material properties definition
		this->diffusion_coefficient = diffusion;
		this->specularity_coefficient = specularity;
		this->shininess_coefficient = shininess;
		this->emission_coefficient = emmision;
		this->materialColour = materialColour;

		//.obj file reading and data stocking
		std::ifstream infile(path);
		//TODO Error management
		std::string line;
		while (std::getline(infile, line))
		{
			std::istringstream iss(line);
			std::string indice;
			iss >> indice;
			//std::cout << "indice : " << indice << std::endl;
			if (indice == "v") {
				float x, y, z;
				iss >> x >> y >> z;
				positions.push_back(glm::vec3(x, y, z));

			}
			else if (indice == "vn") {
				float x, y, z;
				iss >> x >> y >> z;
				normals.push_back(glm::vec3(x, y, z));
			}
			else if (indice == "vt") {
				float u, v;
				iss >> u >> v;
				textures.push_back(glm::vec2(u, v));
			}
			else if (indice == "f") {
				std::string f1, f2, f3;
				iss >> f1 >> f2 >> f3;

				std::string p, t, n;

				//for face 1
				Vertex v1;

				p = f1.substr(0, f1.find("/"));
				f1.erase(0, f1.find("/") + 1);

				t = f1.substr(0, f1.find("/"));
				f1.erase(0, f1.find("/") + 1);

				n = f1.substr(0, f1.find("/"));


				v1.Position = positions.at(std::stof(p) - 1);
				v1.Normal = normals.at(std::stof(n) - 1);
				v1.Texture = textures.at(std::stof(t) - 1);
				vertices.push_back(v1);

				//for face 12
				Vertex v2;

				p = f2.substr(0, f2.find("/"));
				f2.erase(0, f2.find("/") + 1);

				t = f2.substr(0, f2.find("/"));
				f2.erase(0, f2.find("/") + 1);

				n = f2.substr(0, f2.find("/"));


				v2.Position = positions.at(std::stof(p) - 1);
				v2.Normal = normals.at(std::stof(n) - 1);
				v2.Texture = textures.at(std::stof(t) - 1);
				vertices.push_back(v2);

				//for face 3
				Vertex v3;

				p = f3.substr(0, f3.find("/"));
				f3.erase(0, f3.find("/") + 1);

				t = f3.substr(0, f3.find("/"));
				f3.erase(0, f3.find("/") + 1);

				n = f3.substr(0, f3.find("/"));


				v3.Position = positions.at(std::stof(p) - 1);
				v3.Normal = normals.at(std::stof(n) - 1);
				v3.Texture = textures.at(std::stof(t) - 1);
				vertices.push_back(v3);
			}
		}
		//std::cout << positions.size() << std::endl;
		//std::cout << normals.size() << std::endl;
		//std::cout << textures.size() << std::endl;
		std::cout << "Load model with " << vertices.size() << " vertices" << std::endl;

		infile.close();

		numVertices = vertices.size();
	}



	void makeObject(Shader shader, ShaderVFG shaderVFG ,bool texture = true) {
		/* This is a working but not perfect solution, you can improve it if you need/want
		* What happens if you call this function twice on an Model ?
		* What happens when a shader doesn't have a position, tex_coord or normal attribute ?
		*/

		float* data = new float[8 * numVertices];
		for (int i = 0; i < numVertices; i++) {
			Vertex v = vertices.at(i);
			data[i * 8] = v.Position.x;
			data[i * 8 + 1] = v.Position.y;
			data[i * 8 + 2] = v.Position.z;

			data[i * 8 + 3] = v.Texture.x;
			data[i * 8 + 4] = v.Texture.y;

			data[i * 8 + 5] = v.Normal.x;
			data[i * 8 + 6] = v.Normal.y;
			data[i * 8 + 7] = v.Normal.z;
		}

		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);

		//define VBO and VAO as active buffer and active vertex array
		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * numVertices, data, GL_STATIC_DRAW);

		auto att_pos = glGetAttribLocation(shader.ID, "position");
		glEnableVertexAttribArray(att_pos);
		glVertexAttribPointer(att_pos, 3, GL_FLOAT, false, 8 * sizeof(float), (void*)0);

		auto att_pos_2 = glGetAttribLocation(shaderVFG.ID, "position");
		glEnableVertexAttribArray(att_pos_2);
		glVertexAttribPointer(att_pos_2, 3, GL_FLOAT, false, 8 * sizeof(float), (void*)0);

		if (texture) {
			auto att_tex = glGetAttribLocation(shader.ID, "tex_coord");
			glEnableVertexAttribArray(att_tex);
			glVertexAttribPointer(att_tex, 2, GL_FLOAT, false, 8 * sizeof(float), (void*)(3 * sizeof(float)));

		}

		auto att_col = glGetAttribLocation(shader.ID, "normal");
		glEnableVertexAttribArray(att_col);
		glVertexAttribPointer(att_col, 3, GL_FLOAT, false, 8 * sizeof(float), (void*)(5 * sizeof(float)));

		//desactive the buffer
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		delete[] data;

	}


	void draw_on_bullet_object(Shader shader, btRigidBody* body, glm::vec3 scale) {
		btTransform t;
		body->getMotionState()->getWorldTransform(t); //Get the position of the bullet object
		//btVector3 body_translation = t.getOrigin();	  //and put it the sphere_translation vector
		float mat[16];
		t.getOpenGLMatrix(mat);
		//Changing the model matrix following the sphere_translation
		glm::mat4 model(1.0f);
		for (int i = 0;i < 4;i++) {
			for (int j = 0;j < 4; j++) {
				model[j][i] = mat[4 * j + i];
			};
		};
		model = glm::scale(model, scale);
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
		glBindVertexArray(0);
	}

	void draw_on_bullet_object_vehicle_core(Shader shader, btRaycastVehicle* vehicle, glm::vec3 scale) {
		btTransform t; 
		btTransform chassisWorldTransform = vehicle->getChassisWorldTransform();
		// Taking the position of the chassis in the world
		btVector3 chassisPosition = chassisWorldTransform.getOrigin();
		// Taking the orientation of the chassis in the world
		btQuaternion chassisOrientation = chassisWorldTransform.getRotation();
		// Converting position and orientation in glm::vec3 et glm::quat
		glm::vec3 position = glm::vec3(chassisPosition.getX(), chassisPosition.getY(), chassisPosition.getZ()); 
		glm::quat orientation = glm::quat(chassisOrientation.getZ(), chassisOrientation.getY(), chassisOrientation.getX(), chassisOrientation.getW()); 
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, position);
		model *= glm::mat4_cast(orientation);   // Using glm::mat4_cast to convert quaternion in matrices
		model = glm::scale(model, scale); 
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
	}


	void draw_on_bullet_object_vehicle_wheels(Shader shader, btRaycastVehicle* vehicle,float wheelIndex, glm::vec3 scale) {
		btWheelInfo& wheelInfo = vehicle->getWheelInfo(wheelIndex);
		btVector3 wheelPosition = wheelInfo.m_worldTransform.getOrigin(); 
		btQuaternion wheelOrientation = wheelInfo.m_worldTransform.getRotation();
		glm::vec3 position = glm::vec3(wheelPosition.getX(), wheelPosition.getY(), wheelPosition.getZ());
		glm::quat orientation = glm::quat(wheelOrientation.getW(), wheelOrientation.getX(), wheelOrientation.getY(), wheelOrientation.getZ());
		glm::mat4 model = glm::mat4(1.0f); 
		model = glm::translate(model, position); 
		model *= glm::mat4_cast(orientation); 
		model = glm::scale(model, scale); 
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model); 
		shader.setMatrix4("itM", inverse_model);

		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
	}

	void draw_without_bullet_object(Shader shader, glm::mat4 model) {
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
		glBindVertexArray(0);
	}

	void draw_on_bullet_object_VFG(ShaderVFG shader, btRigidBody* body, glm::vec3 scale) {
		btTransform t;
		body->getMotionState()->getWorldTransform(t); //Get the position of the bullet object
		//btVector3 body_translation = t.getOrigin();	  //and put it the sphere_translation vector
		float mat[16];
		t.getOpenGLMatrix(mat);
		//Changing the model matrix following the sphere_translation
		glm::mat4 model(1.0f);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				model[j][i] = mat[4 * j + i];
			};
		};
		model = glm::scale(model, scale);
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
		glBindVertexArray(0);
	}

	void draw_on_bullet_object_vehicle_core_VFG(ShaderVFG shader, btRaycastVehicle* vehicle, glm::vec3 scale) {
		btTransform t;
		btTransform chassisWorldTransform = vehicle->getChassisWorldTransform();
		btVector3 chassisPosition = chassisWorldTransform.getOrigin();
		btQuaternion chassisOrientation = chassisWorldTransform.getRotation();
		glm::vec3 position = glm::vec3(chassisPosition.getX(), chassisPosition.getY(), chassisPosition.getZ());
		glm::quat orientation = glm::quat(chassisOrientation.getZ(), chassisOrientation.getY(), chassisOrientation.getX(), chassisOrientation.getW());
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, position);
		model *= glm::mat4_cast(orientation); 
		model = glm::scale(model, scale);
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
	}


	void draw_on_bullet_object_vehicle_wheels_VFG(ShaderVFG shader, btRaycastVehicle* vehicle, float wheelIndex, glm::vec3 scale) {
		btWheelInfo& wheelInfo = vehicle->getWheelInfo(wheelIndex);
		btVector3 wheelPosition = wheelInfo.m_worldTransform.getOrigin();
		btQuaternion wheelOrientation = wheelInfo.m_worldTransform.getRotation();
		glm::vec3 position = glm::vec3(wheelPosition.getX(), wheelPosition.getY(), wheelPosition.getZ());
		glm::quat orientation = glm::quat(wheelOrientation.getW(), wheelOrientation.getX(), wheelOrientation.getY(), wheelOrientation.getZ());
		glm::mat4 model = glm::mat4(1.0f);
		model = glm::translate(model, position);
		model *= glm::mat4_cast(orientation);
		model = glm::scale(model, scale);
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
	}

	void draw_without_bullet_object_VFG(ShaderVFG shader, glm::mat4 model) {
		glm::mat4 inverse_model = glm::transpose(glm::inverse(model));
		shader.setMatrix4("M", model);
		shader.setMatrix4("itM", inverse_model);
		shader.setVector3f("materialColour", this->materialColour);
		shader.setFloat("u_diffuse", this->diffusion_coefficient);
		shader.setFloat("u_specular", this->specularity_coefficient);
		shader.setFloat("u_shininess", this->shininess_coefficient);
		shader.setFloat("u_emissive", this->emission_coefficient);
		glBindVertexArray(this->VAO);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
		glBindVertexArray(0);
	}

	void draw_with_reflective_texture(Shader shader, glm::mat4 model, unsigned int cubeMapTexture) {
		shader.setMatrix4("M", model);
		glBindVertexArray(this->VAO);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);
		glDrawArrays(GL_TRIANGLES, 0, numVertices);
		glBindVertexArray(0);
	}

	glm::vec3 getObjectPosition(btRigidBody* body) 
	{
		btTransform t;
		body->getMotionState()->getWorldTransform(t);
		float mat[16];
		t.getOpenGLMatrix(mat); 
		return glm::vec3(mat[12],mat[13],mat[14]);
	}
};
#endif