#include "ParticlePhysics.h"
#include "chrono"
#include <iostream>
#include <algorithm>

using namespace NCL::CSC8503;

SPH::SPH(int inNumParticles, Vector3* PosList)
{
    numParticles = inNumParticles;
    particles = new Particle[numParticles];

    particleRadius = 0.5f;

    smoothingRadius = 5.0f;
    particleSpacing = 2.f;

    SmoothingKernelMultiplier = 5 * (6 / (PI * pow(smoothingRadius / 100, 4)));
    SmoothingKernelDerivativeMultiplier = 5 * (12 / (PI * pow(smoothingRadius / 100, 4)));

    NoGridsX = (fence.right - fence.left / smoothingRadius) + 1;
    NoGridsY = (fence.bottom - fence.top / smoothingRadius) + 1;
    NoGridsZ = (fence.back - fence.front / smoothingRadius) + 1;


    hashLookupTable = new int[NoGridsX * NoGridsY * NoGridsZ];
    resetHashLookupTable();

    GridStart(PosList);
    //randomPositionStart(screenWidth, screenHeight);

}

SPH::~SPH()
{
    //delete hashLookupTable;
    delete[]particles;
}

void::SPH::Update(float dt, Vector3* PosList) {
    /*auto start_time
        = std::chrono::high_resolution_clock::now();*/

    SetParticlesInGridsHashing();
    /*auto SetParticlesInGridsHashing_end_time
        = std::chrono::high_resolution_clock::now();*/

    UpdateDensityandPressureGrid();
    /*auto UpdateDensityandPressureGrid_end_time
        = std::chrono::high_resolution_clock::now();*/

    UpdatePressureAccelerationGrid();
    /*auto UpdatePressureAccelerationGrid_end_time
        = std::chrono::high_resolution_clock::now();*/

    updateParticle(dt, PosList);
    /*auto updateParticle_end_time
        = std::chrono::high_resolution_clock::now();*/

    resetHashLookupTable();
    /*auto resetHashLookupTable_end_time
        = std::chrono::high_resolution_clock::now();

    auto taken_time_SetParticlesInGridsHashing = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            SetParticlesInGridsHashing_end_time - start_time)
        .count();

    auto taken_time_UpdateDensityandPressureGrid = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            UpdateDensityandPressureGrid_end_time - SetParticlesInGridsHashing_end_time)
        .count();

    auto taken_time_UpdatePressureAccelerationGrid = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            UpdatePressureAccelerationGrid_end_time - UpdateDensityandPressureGrid_end_time)
        .count();

    auto taken_time_updateParticle = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            updateParticle_end_time - UpdatePressureAccelerationGrid_end_time)
        .count();

    auto taken_time_resetHashLookupTable = std::chrono::duration_cast<
        std::chrono::milliseconds>(
            resetHashLookupTable_end_time - updateParticle_end_time)
        .count();

    std::cout << ""
        << "gridding execution time: " << taken_time_SetParticlesInGridsHashing
        << "ms \n"
        << "dens execution time: " << taken_time_UpdateDensityandPressureGrid
        << "ms \n"
        << "force execution time : " << taken_time_UpdatePressureAccelerationGrid
        << "ms \n"
        << "update execution time: " << taken_time_updateParticle
        << "ms \n"
        << "reset execution time: " << taken_time_resetHashLookupTable
        << "ms \n";
        */

}

void SPH::randomPositionStart()
{
    /*for (int i = 0; i < numParticles; i++) {
        float x = rand() % (int)screenWidth;
        float y = rand() % (int)screeenHeight;
        particles[i].Position = Vector3(x, y);
    }*/
}

void SPH::GridStart(Vector3* PosList)
{
    Vector3 offsetVec(0, 0,0);


    int particlesPerUnitX = static_cast<int>(std::cbrt(numParticles));
    int particlesPerUnitY = particlesPerUnitX;
    int particlesPerUnitZ = (numParticles + particlesPerUnitX * particlesPerUnitY - 1) / (particlesPerUnitX * particlesPerUnitY);
    float spacing = particleRadius * 2 + particleSpacing;

    for (int i = 0; i < numParticles; i++) {
        int ix = i % particlesPerUnitX;
        int iy = (i / particlesPerUnitX) % particlesPerUnitY;
        int iz = i / (particlesPerUnitX * particlesPerUnitY);

        float x = (ix - (particlesPerUnitX / 2.0f) + 0.5f) * spacing + offsetVec.x + (fence.right - fence.left) / 2 + (std::rand() % 100 / 100.0f) * spacing;
        float y = (iy - (particlesPerUnitY / 2.0f) + 0.5f) * spacing + offsetVec.y + (fence.bottom - fence.top) / 2 + (std::rand() % 100 / 100.0f) * spacing;
        float z = (iz - (particlesPerUnitZ / 2.0f) + 0.5f) * spacing + offsetVec.z + (fence.back - fence.front) / 2 + (std::rand() % 100 / 100.0f) * spacing;

        particles[i].Position = Vector3(x, y, z);
        particles[i].PredictedPosition = Vector3(x, y, z);

        PosList[i] = particles[i].Position;
    }
}

void SPH::updateParticle(float dt, Vector3* PosList)
{

    auto updateParticleProperties = [&](Particle& p) {
        int index = &p - particles;
        p.Acceleration = Vector3();
        p.Velocity += p.PressureAcceleration * dt;
        if (gravityEnabled)
            p.Acceleration += gravity;
        p.Velocity += p.Acceleration * dt;
        p.Position += p.Velocity * dt;
        if (p.Position.y >= fence.bottom - particleRadius) {
            p.Position.y = fence.bottom - (0.0001f + particleRadius);
            p.Velocity.y = -p.Velocity.y * dampingRate;
        }
        if (p.Position.y <= fence.top + particleRadius ) {
            p.Position.y = fence.top + (0.0001f + particleRadius);
            p.Velocity.y = -p.Velocity.y * dampingRate;
        }
        if (p.Position.x >= fence.right - particleRadius) {
            p.Position.x = fence.right - (0.0001f + particleRadius);
            p.Velocity.x = -p.Velocity.x * dampingRate;
        }
        if (p.Position.x <= fence.left + particleRadius ) {
            p.Position.x = fence.left + (0.0001f + particleRadius);
            p.Velocity.x = -p.Velocity.x * dampingRate;
        }
        if (p.Position.z >= fence.back - particleRadius ) {
            p.Position.z = fence.back - (0.0001f + particleRadius);
            p.Velocity.z = -p.Velocity.z * dampingRate;
        }
        if (p.Position.z <= fence.front + particleRadius ) {
            p.Position.z = fence.front+ (0.0001f + particleRadius);
            p.Velocity.z = -p.Velocity.z * dampingRate;
        }

        PosList[index] = p.Position;

        p.PredictedPosition = p.Position + p.Velocity * (1 / 30.0f) + p.Acceleration *0.5f * (1 /30.0f) * (1 / 30.0f);

        p.PredictedPosition.x = std::clamp(p.PredictedPosition.x, (float)fence.left, (float)fence.right);
        p.PredictedPosition.y = std::clamp(p.PredictedPosition.y, (float)fence.top, (float)fence.bottom);
        p.PredictedPosition.z = std::clamp(p.PredictedPosition.z, (float)fence.front, (float)fence.back);
    };

    std::for_each(std::execution::par_unseq,
        particles, particles + numParticles,
        updateParticleProperties);

}

double SPH::calcDensityGrid(int particleIndex, Vector3 gridPos)
{
    double density = 0;

    for (auto offset : offsetsGrids) {
        int key = cellHash(gridPos.x + offset.x, gridPos.y + offset.y, gridPos.z + offset.z);
        int startIndex = hashLookupTable[key];
        for (int i = startIndex; i < numParticles; i++) {
            if (key != particles[i].Gridhash) break;

            float dst = (particles[i].PredictedPosition - particles[particleIndex].PredictedPosition).Length();
            double influence = smoothingKernel(smoothingRadius, dst);
            density += mass * influence;

        }
    }

    return density;
}

Vector3 SPH::calcPressureForceGrid(int particleIndex, Vector3 gridPos)
{
    Vector3 pressureForce = Vector3(0, 0, 0);
    //std::vector<Vector3> offsets = gridsys[gridPos.x][gridPos.y]->offsetGrids;

    for (auto offset : offsetsGrids) {
        int key = cellHash(gridPos.x + offset.x, gridPos.y + offset.y, gridPos.z + offset.z);
        int startIndex = hashLookupTable[key];
        for (int i = startIndex; i < numParticles; i++) {
            if (key != particles[i].Gridhash) break;

            if (particleIndex == i) continue;

            Vector3 offsetvec(particles[i].PredictedPosition - particles[particleIndex].PredictedPosition);

            float dst = offsetvec.Length();
            if (dst > smoothingRadius) continue;
            Vector3 dir = dst == 0 ? GetRandomDir() : (offsetvec) / dst;
            double m_slope = smoothingKernerDerivative(smoothingRadius, dst);
            double m_density = particles[i].density;
            double sharedPressure = (particles[i].pressure + particles[particleIndex].pressure) / 2;
            pressureForce += dir * (float)(sharedPressure * m_slope * mass / m_density);

            //add Viscoscity
            Vector3 velocityDiff = particles[i].Velocity - particles[particleIndex].Velocity;
            pressureForce += velocityDiff * viscosityMultiplier * (float)(-m_slope / (m_density * 100));

        }
    }

    return pressureForce;
}

void SPH::UpdateDensityandPressureGrid()
{

    auto calculateDensityAndPressure = [&](Particle& p) {
        p.density = calcDensityGrid(&p - particles, p.GridPos); // Assuming calcDensityGrid takes a particle object
        p.pressure = ConvertDensityToPressure(p.density);
        };

    std::for_each(std::execution::par_unseq,
        particles, particles + numParticles,
        calculateDensityAndPressure);

}

void SPH::UpdatePressureAccelerationGrid()
{
    auto calculatePressureAcceleration = [&](Particle& p) {
        int i = &p - particles;
        particles[i].PressureAcceleration = calcPressureForceGrid(i, particles[i].GridPos);
        };

    std::for_each(std::execution::par_unseq,
        particles, particles + numParticles,
        calculatePressureAcceleration);

}

void SPH::SetParticlesInGridsHashing()
{
    std::for_each(std::execution::par_unseq,
        particles, particles + numParticles,
        [&](Particle& p) {
            int gridX = (p.PredictedPosition.x / smoothingRadius);
            int gridZ = (p.PredictedPosition.z / smoothingRadius);
            int gridY = (p.PredictedPosition.y / smoothingRadius);
            p.Gridhash = cellHash(gridX, gridY,gridZ);
            p.GridPos = Vector3(gridX, gridY,gridZ);
        });

    //sort particles according to hash
    std::sort(std::execution::par, particles, particles + numParticles, [](const Particle& a, const Particle& b) {
        return a.Gridhash < b.Gridhash;
        });

    //create hash lookup for faster navigation
    hashLookupTable[particles[0].Gridhash] = 0;
    for (int i = 1; i < numParticles; i++) {
        if (particles[i].Gridhash != particles[i - 1].Gridhash)
        {
            hashLookupTable[particles[i].Gridhash] = i;
        }
    }
}


