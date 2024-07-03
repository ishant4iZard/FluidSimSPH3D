#include "ParticlePhysics.h"
#include "chrono"
#include <iostream>
#include <algorithm>

using namespace NCL::CSC8503;

SPH::SPH(int inNumParticles, Vector3* PosList)
{
    numParticles = inNumParticles;
    particles.resize(numParticles);

    particleRadius = 0.5f;

    smoothingRadius = 5.0f;
    particleSpacing = 2.f;

    SmoothingKernelMultiplier = 5 * (6 / (PI * pow(smoothingRadius / 100, 4)));
    SmoothingKernelDerivativeMultiplier = 5 * (12 / (PI * pow(smoothingRadius / 100, 4)));



    hashLookupTable = std::vector<int>(numParticles, INT_MAX);
    resetHashLookupTable();

    GridStart(PosList);
    //randomPositionStart(screenWidth, screenHeight);

    glGenBuffers(1, &particleBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, particleBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, particles.size() * sizeof(Particle), particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);

    // Create sortedBuffer without initializing data (just allocate space)
    glGenBuffers(1, &hashLookupBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, hashLookupBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, numParticles * sizeof(int), hashLookupTable.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, hashLookupBuffer);

    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &local_size_x);

    //glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, particles.size() * sizeof(Particle), particles.data());

    if (local_size_x > 1024)local_size_x = 1024;
    //local_size_x = 256;
}

SPH::~SPH()
{
    //delete hashLookupTable;
    //delete[]particles;
}

void::SPH::Update(float dt, Vector3* PosList) {
    auto start_time
        = std::chrono::high_resolution_clock::now();

    //SetParticlesInGridsHashing();
    SetParticlesInGridsHashingGPU();
    auto SetParticlesInGridsHashing_end_time
        = std::chrono::high_resolution_clock::now();

    //UpdateDensityandPressureGrid();
    UpdateDensityandPressureGridGPU();
    auto UpdateDensityandPressureGrid_end_time
        = std::chrono::high_resolution_clock::now();

    //UpdatePressureAccelerationGrid();
    UpdatePressureAccelerationGridGPU();
    auto UpdatePressureAccelerationGrid_end_time
        = std::chrono::high_resolution_clock::now();

    //updateParticle(dt, PosList);
    updateParticleGPU(dt, PosList);
    auto updateParticle_end_time
        = std::chrono::high_resolution_clock::now();

    resetHashLookupTableGPU();
    //resetHashLookupTableGPU();
    auto resetHashLookupTable_end_time
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
        int index = &p - &(particles[0]);
        //p.Acceleration = Vector3();
        p.Velocity += p.PressureAcceleration * dt;
        if (gravityEnabled)
            p.Velocity += gravity * dt;
        //p.Velocity += p.Acceleration * dt;
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

        p.PredictedPosition = p.Position + p.Velocity * (1 / 30.0f) + (gravityEnabled ? gravity : Vector3()) * 0.5f * (1 / 30.0f) * (1 / 30.0f);

        p.PredictedPosition.x = std::clamp(p.PredictedPosition.x, (float)fence.left, (float)fence.right);
        p.PredictedPosition.y = std::clamp(p.PredictedPosition.y, (float)fence.top, (float)fence.bottom);
        p.PredictedPosition.z = std::clamp(p.PredictedPosition.z, (float)fence.front, (float)fence.back);

    };

    std::for_each(std::execution::par_unseq,
        particles.begin(), particles.end(),
        updateParticleProperties);

}



double SPH::calcDensityGrid(int particleIndex, Vector3i gridPos)
{
    double density = 0;

    for (int ix = -1; ix < 2; ix++) {
        for (int iy = -1; iy < 2; iy++) {
            for (int iz = -1; iz < 2; iz++) {
                int key = cellHash(gridPos.x + ix, gridPos.y + iy, gridPos.z + iz);
                int startIndex = hashLookupTable[key];
                for (int i = startIndex; i < numParticles; i++) {
                    if (key != particles[i].Gridhash) break;

                    float dst = (particles[i].PredictedPosition - particles[particleIndex].PredictedPosition).Length();
                    double influence = smoothingKernel(smoothingRadius, dst);
                    density += mass * influence;

                }
            }
        }
    }

    return density;
}

Vector3 SPH::calcPressureForceGrid(int particleIndex, Vector3i gridPos)
{
    Vector3 pressureForce = Vector3(0, 0, 0);

    for (int ix = -1; ix < 2; ix++) {
        for (int iy = -1; iy < 2; iy++) {
            for (int iz = -1; iz < 2; iz++) {
                int key = cellHash(gridPos.x + ix, gridPos.y + iy, gridPos.z + iz);
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
        }
    }

    return pressureForce;
}

void SPH::UpdateDensityandPressureGrid()
{

    auto calculateDensityAndPressure = [&](Particle& p) {
        int index = &p - &(particles[0]);
        Vector3i gridpos = Vector3i(p.Position.x / smoothingRadius , p.Position.y / smoothingRadius, p.Position.z / smoothingRadius);
        p.density = calcDensityGrid(index, gridpos); // Assuming calcDensityGrid takes a particle object
        p.pressure = ConvertDensityToPressure(p.density);
        };

    std::for_each(std::execution::par_unseq,
        particles.begin(), particles.end(),
        calculateDensityAndPressure);

}

void SPH::UpdatePressureAccelerationGrid()
{
    auto calculatePressureAcceleration = [&](Particle& p) {
        int i = &p - &(particles[0]);
        Vector3i gridpos = Vector3i(p.Position.x / smoothingRadius, p.Position.y / smoothingRadius, p.Position.z / smoothingRadius);

        particles[i].PressureAcceleration = calcPressureForceGrid(i, gridpos);
        };

    std::for_each(std::execution::par_unseq,
        particles.begin(), particles.end(),
        calculatePressureAcceleration);

}

void SPH::SetParticlesInGridsHashing()
{
    std::for_each(std::execution::par_unseq,
        particles.begin(), particles.end(),
        [&](Particle& p) {
            int gridX = (p.PredictedPosition.x / smoothingRadius);
            int gridZ = (p.PredictedPosition.z / smoothingRadius);
            int gridY = (p.PredictedPosition.y / smoothingRadius);
            p.Gridhash = cellHash(gridX, gridY,gridZ);
            //p.GridPos = Vector3i(gridX, gridY,gridZ);
        });

    //sort particles according to hash
    std::sort(std::execution::par, particles.begin(), particles.end(), [](const Particle& a, const Particle& b) {
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


void NCL::CSC8503::SPH::SetParticlesInGridsHashingGPU()
{
    glUseProgram(setParticlesInGridsSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);
    glUniform1f(0, smoothingRadius);
    glUniform1i(1, hashX);
    glUniform1i(2, hashY);
    glUniform1i(3, hashZ);
    int dispatchsize = (numParticles + local_size_x - 1) / local_size_x;
    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glUseProgram(parallelSortSource);
    glUniform1i(0, numParticles);

    int l2numparticles = NextPowerOfTwo(numParticles);

    int numStages = static_cast<int>(std::log2(l2numparticles));

    for (int stageIndex = 0; stageIndex < numStages; ++stageIndex) {
        for (int stepIndex = 0; stepIndex <= stageIndex; ++stepIndex) {
            int groupWidth = 1 << (stageIndex - stepIndex);
            int groupHeight = 2 * groupWidth - 1;
            glUniform1i(1, groupWidth);
            glUniform1i(2, groupHeight);
            glUniform1i(3, stepIndex);

            glDispatchCompute((l2numparticles / 2) / 16, 1, 1);
        }
    }
    glMemoryBarrier(GL_ALL_BARRIER_BITS);


    glUseProgram(HashTableSource);
    glUniform1i(0, numParticles);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, hashLookupBuffer);
    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);


}

void NCL::CSC8503::SPH::UpdateDensityandPressureGridGPU()
{
    glUseProgram(updateDensityPressureSource);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, hashLookupBuffer);

    glUniform1f(0, smoothingRadius);
    glUniform1i(1, hashX);
    glUniform1i(2, hashY);
    glUniform1i(3, hashZ);
    glUniform1f(4, mass);
    glUniform1f(5, SmoothingKernelMultiplier);
    glUniform1f(6, targetDensity);
    glUniform1f(7, pressureMultiplier);

    int dispatchsize = (numParticles + local_size_x - 1) / local_size_x;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);


}

void NCL::CSC8503::SPH::UpdatePressureAccelerationGridGPU()
{
    glUseProgram(updatePressureAccelerationSource);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, hashLookupBuffer);

    glUniform1f(0, smoothingRadius);
    glUniform1i(1, hashX);
    glUniform1i(2, hashY);
    glUniform1i(3, hashZ);
    glUniform1f(4, mass);
    glUniform1f(5, SmoothingKernelMultiplier);
    glUniform1f(6, viscosityMultiplier);

    int dispatchsize = (numParticles + local_size_x - 1) / local_size_x;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);


}

void NCL::CSC8503::SPH::updateParticleGPU(float dt, Vector3* PosList)
{
    glUseProgram(updateParticlesSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);

    glUniform1i(0, fence.left);
    glUniform1i(1, fence.right);
    glUniform1i(2, fence.bottom);
    glUniform1i(3, fence.top);
    glUniform1i(4, fence.front);
    glUniform1i(5, fence.back);
    glUniform1f(6, dt);
    glUniform1i(7, gravityEnabled ? 1 : 0);
    glUniform3fv(8, 1, &gravity[0]);
    glUniform1f(9, particleRadius);
    glUniform1f(10, dampingRate);

    int dispatchsize = (numParticles + local_size_x - 1) / local_size_x;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    /*glGetNamedBufferSubData(particleBuffer, 0, particles.size() * sizeof(Particle), particles.data());

    std::for_each(std::execution::par_unseq,
        particles.begin(), particles.end(), [&](const Particle& p) {
            int i = &p - &(particles[0]);
            PosList[i] = particles[i].Position;
        });*/
}

void NCL::CSC8503::SPH::resetHashLookupTableGPU()
{
    glUseProgram(resetHashTableSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, hashLookupBuffer);
    glUniform1i(0, numParticles);

    int dispatchsize = (numParticles + local_size_x - 1) / local_size_x;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

}
