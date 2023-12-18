#ifndef PARTICLE_H
#define PARTICLE_H
#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

#include "shader.h"
#include "camera.h"


// Represents a single particle and its state
struct Particle {
    glm::vec3 Position, Velocity;
    glm::vec4 Color;
    float     Life;

    Particle() : Position(0.0f), Velocity(0.0f), Color(1.0f), Life(0.0f) { }
};


// ParticleGenerator acts as a container for rendering a large number of 
// particles by repeatedly spawning and updating particles and killing 
// them after a given amount of time.
class ParticleGenerator
{
public:
    // constructor
    ParticleGenerator(Shader shader, unsigned int amount, Camera camera)
        : shader(shader), amount(amount)
    {
        this->init();
    }

    // update all particles
    void Update(float dt, float yMax, float far_plane, float gravity, glm::vec3 wind, unsigned int newParticles, Camera camera)
    {
        // add new particles 
        for (unsigned int i = 0; i < newParticles; ++i)
        {
            int unusedParticle = this->firstUnusedParticle();
            this->respawnParticle(this->particles[unusedParticle], camera.Position, yMax, far_plane, gravity, wind);
        }
        // update all particles
        for (unsigned int i = 0; i < this->amount; ++i)
        {
            Particle& p = this->particles[i];
            p.Life = p.Position.y / yMax;
            //p.Color.a = 1.0 - length(p.Position - camera.Position) / far_plane;
            if (p.Life > 0.0f)
            {	// particle is alive, thus update
                p.Position += p.Velocity * dt;
                p.Velocity += glm::vec3(0.0, -gravity, 0.0);
            }
        }
    }

    // render all particles
    void Draw(Camera camera)
    {
        // use additive blending to give it a 'glow' effect
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);
        this->shader.use();
        for (Particle particle : this->particles)
        {
            if (particle.Life > 0.0f)
            {
                glm::mat4 view = camera.GetViewMatrix();
                glm::mat4 perspective = camera.GetProjectionMatrix();
                glm::mat4 model = glm::translate(glm::mat4(1.0), particle.Position);
                model = glm::scale(model, glm::vec3(10.0, 10.0, 10.0));
                this->shader.setMatrix4("M", model);
                this->shader.setMatrix4("V", view);
                this->shader.setMatrix4("P", perspective);
                this->shader.setVector4f("color", particle.Color);
                glBindVertexArray(this->VAO);
                glDrawArrays(GL_TRIANGLES, 0, 6);
                glBindVertexArray(0);
                // Créer un flux de chaînes de caractères (ostringstream)
                std::ostringstream oss;
                std::vector<float> myVector = { particle.Position.x, particle.Position.y , particle.Position.z };
                // Utiliser une boucle pour concaténer les éléments du vecteur dans la chaîne
                for (const auto& element : myVector) {
                    oss << element << " ";
                }
                std::cout << "Draw at " << oss.str() << std::endl;
            }
        }
        // don't forget to reset to default blending mode
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    void init()
    {
        // set up mesh and attribute properties
        unsigned int VBO;
        float particle_quad[] = {
            0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,

            0.0f, 1.0f, 0.0f, 1.0f,
            1.0f, 1.0f, 1.0f, 1.0f,
            1.0f, 0.0f, 1.0f, 0.0f
        };
        glGenVertexArrays(1, &this->VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(this->VAO);
        // fill mesh buffer
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(particle_quad), particle_quad, GL_STATIC_DRAW);
        // set mesh attributes
        auto att_pos = glGetAttribLocation(this->shader.ID, "position");
        glEnableVertexAttribArray(att_pos);
        glVertexAttribPointer(att_pos, 3, GL_FLOAT, false, 8 * sizeof(float), (void*)0);
        glBindVertexArray(0);

        // create this->amount default particle instances
        for (unsigned int i = 0; i < this->amount; ++i)
            this->particles.push_back(Particle());
    }

    // stores the index of the last particle used (for quick access to next dead particle)
    unsigned int lastUsedParticle = 0;
    unsigned int firstUnusedParticle()
    {
        // first search from last used particle, this will usually return almost instantly
        for (unsigned int i = lastUsedParticle; i < this->amount; ++i) {
            if (this->particles[i].Life <= 0.0f) {
                lastUsedParticle = i;
                return i;
            }
        }
        // otherwise, do a linear search
        for (unsigned int i = 0; i < lastUsedParticle; ++i) {
            if (this->particles[i].Life <= 0.0f) {
                lastUsedParticle = i;
                return i;
            }
        }
        // all particles are taken, override the first one (note that if it repeatedly hits this case, more particles should be reserved)
        lastUsedParticle = 0;
        return 0;
    }

    void respawnParticle(Particle& particle, glm::vec3 cameraPosition, float yMax, float far_plane, float gravity, glm::vec3 wind)
    {
        float randomX = ((rand() % 2*far_plane) - far_plane);
        float randomZ = ((rand() % 2 * far_plane) - far_plane);
        float rColor = 0.5f;
        particle.Position = glm::vec3(cameraPosition.x, yMax, cameraPosition.z) + glm::vec3(randomX, 0.0, randomZ);
        particle.Color = glm::vec4(rColor, rColor, rColor, 1.0);
        particle.Life = 1.0f;
        particle.Velocity = glm::vec3(0.0, -gravity, 0.0);
    }

private:
    // state
    std::vector<Particle> particles;
    unsigned int amount;
    // render state
    Shader shader;
    unsigned int VAO;
};

#endif