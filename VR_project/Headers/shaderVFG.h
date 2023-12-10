#ifndef SHADERVFG_H
#define SHADERVFG_H

#include <glad/glad.h>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

class ShaderVFG
{
public:
    GLuint ID;

    ShaderVFG(const char* vertexPath, const char* fragmentPath, const char* geometryPath)
    {
        // 1. retrieve the vertex/fragment source code from filePath
        std::string vertexCode;
        std::string fragmentCode;
        std::string geometryCode;
        std::ifstream vShaderFile;
        std::ifstream fShaderFile;
        std::ifstream gShaderFile;
        // ensure ifstream objects can throw exceptions:
        vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        gShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
        try
        {
            // open files
            vShaderFile.open(vertexPath);
            fShaderFile.open(fragmentPath);
            gShaderFile.open(geometryPath);
            std::stringstream vShaderStream, fShaderStream, gShaderStream;
            // read file's buffer contents into streams
            vShaderStream << vShaderFile.rdbuf();
            fShaderStream << fShaderFile.rdbuf();
            gShaderStream << gShaderFile.rdbuf();
            // close file handlers
            vShaderFile.close();
            fShaderFile.close();
            gShaderFile.close();
            // convert stream into string
            vertexCode = vShaderStream.str();
            fragmentCode = fShaderStream.str();
            geometryCode = gShaderStream.str();
        }
        catch (std::ifstream::failure& e)
        {
            std::cout << "ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ: " << e.what() << std::endl;
        }
        const char* vShaderCode = vertexCode.c_str();
        const char* fShaderCode = fragmentCode.c_str();
        const char* gShaderCode = geometryCode.c_str();

        GLuint vertex = compileShader(vertexCode, GL_VERTEX_SHADER);
        GLuint fragment = compileShader(fragmentCode, GL_FRAGMENT_SHADER);
        GLuint geometry = compileShader(geometryCode, GL_GEOMETRY_SHADER);
        ID = compileProgram(vertex, fragment, geometry);
    }

    ShaderVFG(std::string vShaderCode, std::string fShaderCode, std::string gShaderCode)
    {
        GLuint vertex = compileShader(vShaderCode, GL_VERTEX_SHADER);
        GLuint fragment = compileShader(fShaderCode, GL_FRAGMENT_SHADER);
        GLuint geometry = compileShader(gShaderCode, GL_GEOMETRY_SHADER);
        ID = compileProgram(vertex, fragment, geometry);
    }

    void use() {
        glUseProgram(ID);
    }
    void setInteger(const GLchar* name, GLint value) {
        glUniform1i(glGetUniformLocation(ID, name), value);
    }
    void setFloat(const GLchar* name, GLfloat value) {
        glUniform1f(glGetUniformLocation(ID, name), value);
    }
    void setVector3f(const GLchar* name, GLfloat x, GLfloat y, GLfloat z) {
        glUniform3f(glGetUniformLocation(ID, name), x, y, z);
    }
    void setVector3f(const GLchar* name, const glm::vec3& value) {
        glUniform3f(glGetUniformLocation(ID, name), value.x, value.y, value.z);
    }
    void setMatrix4(const GLchar* name, const glm::mat4& matrix) {
        glUniformMatrix4fv(glGetUniformLocation(ID, name), 1, GL_FALSE, glm::value_ptr(matrix));
    }

private:
    GLuint compileShader(std::string shaderCode, GLenum shaderType)
    {
        GLuint shader = glCreateShader(shaderType);
        const char* code = shaderCode.c_str();
        glShaderSource(shader, 1, &code, NULL);
        glCompileShader(shader);

        GLchar infoLog[1024];
        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success)
        {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::string t = "undetermined";
            if (shaderType == GL_VERTEX_SHADER) {
                t = "vertex shader";
            }
            else if (shaderType == GL_FRAGMENT_SHADER) {
                t = "fragment shader";
            }
            std::cout << "ERROR::SHADER_COMPILATION_ERROR of the " << t << ": " << shaderType << infoLog << std::endl;
        }
        return shader;
    }

    GLuint compileProgram(GLuint vertexShader, GLuint fragmentShader, GLuint geometryShader)
    {
        GLuint programID = glCreateProgram();

        glAttachShader(programID, vertexShader);
        glAttachShader(programID, fragmentShader);
        glAttachShader(programID, geometryShader);
        glLinkProgram(programID);


        GLchar infoLog[1024];
        GLint success;
        glGetProgramiv(programID, GL_LINK_STATUS, &success);
        if (!success)
        {
            glGetProgramInfoLog(programID, 1024, NULL, infoLog);
            std::cout << "ERROR::PROGRAM_LINKING_ERROR:  " << infoLog << std::endl;
        }
        return programID;
    }

};
#endif
