#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        auto d_vec = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; ++i)
        {
            masses.push_back(new Mass(start + d_vec * i, node_mass, false));
            if (i != 0){
                springs.push_back(new Spring(masses[i-1], masses[i], k));
            }
        }
        for (int i: pinned_nodes){
            masses[i] -> pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        springs[0]->m1->forces = Vector2D();
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            auto a = s->m1, b = s->m2;
            b->forces = Vector2D();
            auto l = (a->start_position - b->start_position).norm();
            auto a_to_b = b->position - a->position;
            auto curr_l = a_to_b.norm();
            auto force_a = s->k * a_to_b * (curr_l - l) / curr_l;
            a->forces += force_a;
            b->forces -= force_a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                auto accl = m->forces / m->mass;
                auto vel_old = m->velocity;
                auto damping = 1 - 0.0001;
                
                m->velocity += accl * delta_t;
                 m->velocity *= damping; // global damping
                m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        springs[0]->m1->forces = Vector2D();
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            auto a = s->m1, b = s->m2;
            
            b->forces = Vector2D();
            auto l = (a->start_position - b->start_position).norm();
            auto a_to_b = b->position - a->position;
            auto curr_l = a_to_b.norm();
            auto force_a = s->k * a_to_b * (curr_l - l) / curr_l;
            a->forces += force_a;
            b->forces -= force_a;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += m->mass * gravity;
                auto accl = m->forces / m->mass;
                // std::cout << m->last_position << "\n "<< m->position << std::endl;
                auto damping = 1 - 0.0001;
                m->position += (accl * delta_t * delta_t + m->position - m->last_position) * damping;
                m->last_position = temp_position;
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
