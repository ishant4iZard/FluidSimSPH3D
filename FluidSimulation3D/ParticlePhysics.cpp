#include "ParticlePhysics.h"
#include "chrono"
#include "MarchingCubesConstants.h"
#include "Debug.h"
#include <iostream>
#include <algorithm>

using namespace NCL::CSC8503;

SPH::SPH(GameWorld& ingameWorld) : gameWorld(ingameWorld) {
    InitializeParticles();
    InitializeMarchingCubesVariables();
    InitializeHashingAndSortingVariables();
    GridStart(); // this function is called before InitializeOpenGLBuffers so that particle data will be set properly while initalising buffer
    InitializeOpenGLBuffers();

    m_fenceEdges = calculateEdges(m_fence);
    m_isRenderParticles = false;
    m_isRenderSurface = true;
}

SPH::~SPH()
{
    glDeleteBuffers(1, &m_particleBuffer);
    glDeleteBuffers(1, &m_hashLookupBuffer);
    glDeleteBuffers(1, &m_neighbourParticlesBuffer);
    glDeleteBuffers(1, &m_triangleBuffer);
    glDeleteBuffers(1, &m_counterBuffer);
    glDeleteBuffers(1, &m_edgeTableBuffer);
    glDeleteBuffers(1, &m_triTableBuffer);
    glDeleteBuffers(1, &m_maxHeightBuffer);

    glDeleteProgram(setParticlesInGridsSource);
    glDeleteProgram(parallelSortSource);
    glDeleteProgram(HashTableSource);
    glDeleteProgram(updateDensityPressureSource);
    glDeleteProgram(updatePressureAccelerationSource);
    glDeleteProgram(updateParticlesSource);
    glDeleteProgram(resetHashTableSource);
    glDeleteProgram(preMarchingCubesSource);
    glDeleteProgram(MarchingCubesSource);

    glFinish();
}

void SPH::InitializeParticles() {
    m_numParticles = 500000;
    m_particles.resize(m_numParticles);

    m_particleRadius = 0.5f;
    m_particleSmoothingRadius = 5.0f;
    m_particleSpacing = 2.f;
    m_particleMass = 1.0f;
    m_particleDampingRate = 0.98f;
    m_isParticleGravityEnabled = 1;
    m_particleGravity = Vector3(0.f, -9.8f, 0.f);
    m_particleTargetDensity = 50.f;
    m_particlePressureMultiplier = 1.f;
    m_particleViscosityMultiplier = 0.7f;

    m_particleSmoothingKernelMultiplier = 5 * (6 / (PI * pow(m_particleSmoothingRadius / 10, 4)));
    m_particleSmoothingKernelDerivativeMultiplier = 5 * (12 / (PI * pow(m_particleSmoothingRadius / 10, 4)));
}

void SPH::InitializeMarchingCubesVariables() {
    m_marchingCubesSize = 2.f;
    m_numCubesXaxisMarchingCubes = floor((m_fence.right - m_fence.left) / m_marchingCubesSize) + 2;
    m_numCubesYaxisMarchingCubes = floor((m_fence.top - m_fence.bottom) / m_marchingCubesSize) + 2;
    m_numCubesZaxisMarchingCubes = floor((m_fence.back - m_fence.front) / m_marchingCubesSize) + 2;
    m_marchingCubesNoGrids = m_numCubesXaxisMarchingCubes * m_numCubesYaxisMarchingCubes * m_numCubesZaxisMarchingCubes;
    m_marchingCubesIsoLevel = 3;
}

void SPH::InitializeHashingAndSortingVariables() {
    l2numparticles = NextPowerOfTwo(m_numParticles);
    numStages = static_cast<int>(std::log2(l2numparticles));
    m_sortingLocalSizeX = 16;

    m_hashLookupTable = std::vector<int>(1620235, INT_MAX);
    resetHashLookupTable();
}

void SPH::InitializeOpenGLBuffers() {
    glGenBuffers(1, &m_particleBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_particleBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_numParticles * sizeof(Particle), m_particles.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);

    glGenBuffers(1, &m_hashLookupBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_hashLookupBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, m_hashLookupTable.size() * sizeof(int), m_hashLookupTable.data(), GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);

    glGenBuffers(1, &m_neighbourParticlesBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_neighbourParticlesBuffer);
    glBufferStorage(GL_SHADER_STORAGE_BUFFER, m_marchingCubesNoGrids * sizeof(int), nullptr, GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_neighbourParticlesBuffer);

    glGenBuffers(1, &m_triangleBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_triangleBuffer);
    glBufferStorage(GL_SHADER_STORAGE_BUFFER, floor(7000000 / pow(m_marchingCubesSize, 2)) * 3 * sizeof(Vector4), nullptr, GL_DYNAMIC_STORAGE_BIT);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_triangleBuffer);

    glGenBuffers(1, &m_counterBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_counterBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(unsigned int), &m_numTriMarchingCubes, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_counterBuffer);

    glGenBuffers(1, &m_edgeTableBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_edgeTableBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 256 * sizeof(int), &edgeTable, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_edgeTableBuffer);

    glGenBuffers(1, &m_triTableBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_triTableBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 256 * 16 * sizeof(int), &triTable, GL_STATIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_triTableBuffer);

    glGenBuffers(1, &m_maxHeightBuffer);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_maxHeightBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int), &m_maxParticleHeight, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, m_maxHeightBuffer);

    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &m_localSizeX);

    int localSizeBest = ((int)sqrt(m_numParticles) / 32) * 32;
    if (m_localSizeX > localSizeBest) m_localSizeX = localSizeBest;
}

void::SPH::Update(float dt) {
    if (nextHashingFrame == 0) {
        SetParticlesInGridsHashingGPU();
        nextHashingFrame = kHashFrameInterval;
    }
    else {
        nextHashingFrame--;
    }

    UpdateDensityandPressureGridGPU();
    UpdatePressureAccelerationGridGPU();
    updateParticleGPU(dt);

    if (m_isRenderSurface) {
        PreMarchingCubes();
        MarchingCubes();
    }

    for (int i = 0; i < m_fenceEdges.size(); i++) {
        Debug::DrawLine(m_fenceEdges[i].first, m_fenceEdges[i].second);
    }
            
    gameWorld.marchingCubestriangleCount = m_numTriMarchingCubes;

    //redacted code
    //SetParticlesInGridsHashing();
    //UpdateDensityandPressureGrid();
    //UpdatePressureAccelerationGrid();
    //updateParticle(dt, PosList);
}

void SPH::GridStart()
{
    Vector3 offsetVec(0, 0,0);


    int particlesPerUnitX = static_cast<int>(std::cbrt(m_numParticles));
    int particlesPerUnitY = particlesPerUnitX;
    int particlesPerUnitZ = (m_numParticles + particlesPerUnitX * particlesPerUnitY - 1) / (particlesPerUnitX * particlesPerUnitY);
    float spacing = m_particleRadius * 2 + m_particleSpacing;

    std::for_each(std::execution::par_unseq, m_particles.begin(), m_particles.end(), [=](Particle& particle) mutable {
        int i = &particle - &m_particles[0];  // Index of the current particle

        int ix = i % particlesPerUnitX;
        int iy = (i / particlesPerUnitX) % particlesPerUnitY;
        int iz = i / (particlesPerUnitX * particlesPerUnitY);

        float x = (ix - (particlesPerUnitX / 2.0f) + 0.5f) * spacing + offsetVec.x + (m_fence.right - m_fence.left) / 2 + (std::rand() % 100 / 100.0f) * spacing;
        float y = (iy - (particlesPerUnitY / 2.0f) + 0.5f) * spacing + offsetVec.y + (m_fence.top - m_fence.bottom) / 2 + (std::rand() % 100 / 100.0f) * spacing;
        float z = (iz - (particlesPerUnitZ / 2.0f) + 0.5f) * spacing + offsetVec.z + (m_fence.back - m_fence.front) / 2 + (std::rand() % 100 / 100.0f) * spacing;

        particle.Position = Vector3(x, y, z);
        particle.PredictedPosition = Vector3(x, y, z);
        });
}

#pragma region GPUBasedFunction

void NCL::CSC8503::SPH::SetParticlesInGridsHashingGPU()
{
    glUseProgram(setParticlesInGridsSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glUniform1f(0, m_particleSmoothingRadius);
    //glUniform1i(1, kHashX);
    //glUniform1i(2, kHashY);
    //glUniform1i(3, kHashZ);
    glUniform1i(4, m_hashLookupTable.size());
    int dispatchsize = (m_numParticles + m_localSizeX - 1) / m_localSizeX;
    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glUseProgram(parallelSortSource);
    glUniform1i(0, m_numParticles);

    for (int stageIndex = 0; stageIndex < numStages; ++stageIndex) {
        for (int stepIndex = 0; stepIndex <= stageIndex; ++stepIndex) {
            int groupWidth = 1 << (stageIndex - stepIndex);
            int groupHeight = 2 * groupWidth - 1;
            glUniform1i(1, groupWidth);
            glUniform1i(2, groupHeight);
            glUniform1i(3, stepIndex);

            glDispatchCompute((l2numparticles /2) / m_sortingLocalSizeX, 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        }
    }
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    resetHashLookupTableGPU();

    glUseProgram(HashTableSource);
    glUniform1i(0, m_numParticles);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);
    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void NCL::CSC8503::SPH::UpdateDensityandPressureGridGPU()
{
    glUseProgram(updateDensityPressureSource);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);

    glUniform1f(0, m_particleSmoothingRadius);
    //glUniform1i(1, kHashX);
    //glUniform1i(2, kHashY);
    //glUniform1i(3, kHashZ);
    glUniform1f(4, m_particleMass);
    glUniform1f(5, m_particleSmoothingKernelMultiplier);
    glUniform1f(6, m_particleTargetDensity);
    glUniform1f(7, m_particlePressureMultiplier);

    int dispatchsize = (m_numParticles + m_localSizeX - 1) / m_localSizeX;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void NCL::CSC8503::SPH::UpdatePressureAccelerationGridGPU()
{
    glUseProgram(updatePressureAccelerationSource);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);

    glUniform1f(0, m_particleSmoothingRadius);
    //glUniform1i(1, kHashX);
    //glUniform1i(2, kHashY);
    //glUniform1i(3, kHashZ);
    glUniform1f(4, m_particleMass);
    glUniform1f(5, m_particleSmoothingKernelMultiplier);
    glUniform1f(6, m_particleViscosityMultiplier);

    int dispatchsize = (m_numParticles + m_localSizeX - 1) / m_localSizeX;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void NCL::CSC8503::SPH::updateParticleGPU(float dt)
{
    m_maxParticleHeight = 0;
    glNamedBufferSubData(m_maxHeightBuffer, 0, sizeof(int), &m_maxParticleHeight);

    glUseProgram(updateParticlesSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, m_maxHeightBuffer);

    glUniform1i(0, m_fence.left);
    glUniform1i(1, m_fence.right);
    glUniform1i(2, m_fence.bottom);
    glUniform1i(3, m_fence.top);
    glUniform1i(4, m_fence.front);
    glUniform1i(5, m_fence.back);
    glUniform1f(6, dt);
    glUniform1i(7, m_isParticleGravityEnabled ? 1 : 0);
    glUniform3fv(8, 1, &m_particleGravity[0]);
    glUniform1f(9, m_particleRadius);
    glUniform1f(10, m_particleDampingRate);
    glUniform1f(11, m_particleSmoothingRadius);

    int dispatchsize = (m_numParticles + m_localSizeX - 1) / m_localSizeX;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void NCL::CSC8503::SPH::resetHashLookupTableGPU()
{
    glUseProgram(resetHashTableSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);
    glUniform1i(0, m_numParticles);

    int dispatchsize = (m_numParticles + m_localSizeX - 1) / m_localSizeX;

    glDispatchCompute(dispatchsize, 1, 1);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

}

void NCL::CSC8503::SPH::PreMarchingCubes()
{
    glUseProgram(preMarchingCubesSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_particleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, m_hashLookupBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_neighbourParticlesBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, m_maxHeightBuffer);

    glUniform1f(0, m_particleSmoothingRadius);
    //glUniform1i(1, kHashX);
    //glUniform1i(2, kHashY);
    //glUniform1i(3, kHashZ);
    glUniform1f(4, m_marchingCubesSize);
    glUniform1i(5, m_fence.left);
    glUniform1i(6, m_fence.right);
    glUniform1i(7, m_fence.bottom);
    glUniform1i(8, m_fence.top);
    glUniform1i(9, m_fence.front);
    glUniform1i(10, m_fence.back);
    glUniform3i(11, m_numCubesXaxisMarchingCubes, m_numCubesYaxisMarchingCubes, m_numCubesZaxisMarchingCubes);

    glDispatchCompute((m_numCubesXaxisMarchingCubes + 7) / 8, (m_numCubesYaxisMarchingCubes + 7) / 8, (m_numCubesZaxisMarchingCubes + 7) / 8);

    glMemoryBarrier(GL_ALL_BARRIER_BITS);
}

void NCL::CSC8503::SPH::MarchingCubes()
{
    m_numTriMarchingCubes = 0;
    glNamedBufferSubData(m_counterBuffer, 0, sizeof(unsigned int), &m_numTriMarchingCubes);

    glUseProgram(MarchingCubesSource);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, m_neighbourParticlesBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, m_triangleBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, m_counterBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, m_edgeTableBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, m_triTableBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, m_maxHeightBuffer);

    glUniform1f(0, m_marchingCubesIsoLevel);
    glUniform1f(1, m_marchingCubesSize);
    glUniform1i(2, m_numCubesXaxisMarchingCubes);
    glUniform1i(3, m_numCubesYaxisMarchingCubes);
    glUniform1i(4, m_numCubesZaxisMarchingCubes);

    glDispatchCompute((m_numCubesXaxisMarchingCubes + 7) / 8, (m_numCubesYaxisMarchingCubes + 7) / 8, (m_numCubesZaxisMarchingCubes + 7) / 8);

    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    glGetNamedBufferSubData(m_counterBuffer, 0, sizeof(unsigned int), &m_numTriMarchingCubes);
}
#pragma endregion

//redacted code
#pragma region CPUBasedFunction
//
//void SPH::updateParticle(float dt)
//{
//
//    auto updateParticleProperties = [&](Particle& p) {
//        int index = &p - &(m_particles[0]);
//        //p.Acceleration = Vector3();
//        p.Velocity += p.PressureAcceleration * dt;
//        if (m_isParticleGravityEnabled)
//            p.Velocity += m_particleGravity * dt;
//        //p.Velocity += p.Acceleration * dt;
//        p.Position += p.Velocity * dt;
//        if (p.Position.y >= m_fence.top - m_particleRadius) {
//            p.Position.y = m_fence.top - (0.0001f + m_particleRadius);
//            p.Velocity.y = -p.Velocity.y * m_particleDampingRate;
//        }
//        if (p.Position.y <= m_fence.bottom + m_particleRadius ) {
//            p.Position.y = m_fence.bottom + (0.0001f + m_particleRadius);
//            p.Velocity.y = -p.Velocity.y * m_particleDampingRate;
//        }
//        if (p.Position.x >= m_fence.right - m_particleRadius) {
//            p.Position.x = m_fence.right - (0.0001f + m_particleRadius);
//            p.Velocity.x = -p.Velocity.x * m_particleDampingRate;
//        }
//        if (p.Position.x <= m_fence.left + m_particleRadius ) {
//            p.Position.x = m_fence.left + (0.0001f + m_particleRadius);
//            p.Velocity.x = -p.Velocity.x * m_particleDampingRate;
//        }
//        if (p.Position.z >= m_fence.back - m_particleRadius ) {
//            p.Position.z = m_fence.back - (0.0001f + m_particleRadius);
//            p.Velocity.z = -p.Velocity.z * m_particleDampingRate;
//        }
//        if (p.Position.z <= m_fence.front + m_particleRadius ) {
//            p.Position.z = m_fence.front+ (0.0001f + m_particleRadius);
//            p.Velocity.z = -p.Velocity.z * m_particleDampingRate;
//        }
//
//        p.PredictedPosition = p.Position + p.Velocity * (1 / 30.0f) + (m_isParticleGravityEnabled ? m_particleGravity : Vector3()) * 0.5f * (1 / 30.0f) * (1 / 30.0f);
//
//        p.PredictedPosition.x = std::clamp(p.PredictedPosition.x, (float)m_fence.left, (float)m_fence.right);
//        p.PredictedPosition.y = std::clamp(p.PredictedPosition.y, (float)m_fence.bottom, (float)m_fence.top);
//        p.PredictedPosition.z = std::clamp(p.PredictedPosition.z, (float)m_fence.front, (float)m_fence.back);
//
//    };
//
//    std::for_each(std::execution::par_unseq,
//        m_particles.begin(), m_particles.end(),
//        updateParticleProperties);
//
//}
//
//double SPH::calcDensityGrid(int particleIndex, Vector3i gridPos)
//{
//    double density = 0;
//
//    for (int ix = -1; ix < 2; ix++) {
//        for (int iy = -1; iy < 2; iy++) {
//            for (int iz = -1; iz < 2; iz++) {
//                int key = cellHash(gridPos.x + ix, gridPos.y + iy, gridPos.z + iz);
//                int startIndex = m_hashLookupTable[key];
//                for (int i = startIndex; i < m_numParticles; i++) {
//                    if (key != m_particles[i].Gridhash) break;
//
//                    float dst = (m_particles[i].PredictedPosition - m_particles[particleIndex].PredictedPosition).Length();
//                    double influence = smoothingKernel(m_particleSmoothingRadius, dst);
//                    density += m_particleMass * influence;
//
//                }
//            }
//        }
//    }
//
//    return density;
//}
//
//Vector3 SPH::calcPressureForceGrid(int particleIndex, Vector3i gridPos)
//{
//    Vector3 pressureForce = Vector3(0, 0, 0);
//
//    for (int ix = -1; ix < 2; ix++) {
//        for (int iy = -1; iy < 2; iy++) {
//            for (int iz = -1; iz < 2; iz++) {
//                int key = cellHash(gridPos.x + ix, gridPos.y + iy, gridPos.z + iz);
//                int startIndex = m_hashLookupTable[key];
//                for (int i = startIndex; i < m_numParticles; i++) {
//                    if (key != m_particles[i].Gridhash) break;
//
//                    if (particleIndex == i) continue;
//
//                    Vector3 offsetvec(m_particles[i].PredictedPosition - m_particles[particleIndex].PredictedPosition);
//
//                    float dst = offsetvec.Length();
//                    if (dst > m_particleSmoothingRadius) continue;
//                    Vector3 dir = dst == 0 ? GetRandomDir() : (offsetvec) / dst;
//                    double m_slope = smoothingKernerDerivative(m_particleSmoothingRadius, dst);
//                    double m_density = m_particles[i].density;
//                    double sharedPressure = (m_particles[i].pressure + m_particles[particleIndex].pressure) / 2;
//                    pressureForce += dir * (float)(sharedPressure * m_slope * m_particleMass / m_density);
//
//                    //add Viscoscity
//                    Vector3 velocityDiff = m_particles[i].Velocity - m_particles[particleIndex].Velocity;
//                    pressureForce += velocityDiff * m_particleViscosityMultiplier * (float)(-m_slope / (m_density * 100));
//
//                }
//            }
//        }
//    }
//
//    return pressureForce;
//}
//
//void SPH::UpdateDensityandPressureGrid()
//{
//
//    auto calculateDensityAndPressure = [&](Particle& p) {
//        int index = &p - &(m_particles[0]);
//        Vector3i gridpos = Vector3i(p.Position.x / m_particleSmoothingRadius , p.Position.y / m_particleSmoothingRadius, p.Position.z / m_particleSmoothingRadius);
//        p.density = calcDensityGrid(index, gridpos); // Assuming calcDensityGrid takes a particle object
//        p.pressure = ConvertDensityToPressure(p.density);
//        };
//
//    std::for_each(std::execution::par_unseq,
//        m_particles.begin(), m_particles.end(),
//        calculateDensityAndPressure);
//
//}
//
//void SPH::UpdatePressureAccelerationGrid()
//{
//    auto calculatePressureAcceleration = [&](Particle& p) {
//        int i = &p - &(m_particles[0]);
//        Vector3i gridpos = Vector3i(p.Position.x / m_particleSmoothingRadius, p.Position.y / m_particleSmoothingRadius, p.Position.z / m_particleSmoothingRadius);
//
//        m_particles[i].PressureAcceleration = calcPressureForceGrid(i, gridpos);
//        };
//
//    std::for_each(std::execution::par_unseq,
//        m_particles.begin(), m_particles.end(),
//        calculatePressureAcceleration);
//
//}
//
//void SPH::SetParticlesInGridsHashing()
//{
//    std::for_each(std::execution::par_unseq,
//        m_particles.begin(), m_particles.end(),
//        [&](Particle& p) {
//            int gridX = (p.PredictedPosition.x / m_particleSmoothingRadius);
//            int gridZ = (p.PredictedPosition.z / m_particleSmoothingRadius);
//            int gridY = (p.PredictedPosition.y / m_particleSmoothingRadius);
//            p.Gridhash = cellHash(gridX, gridY,gridZ);
//            //p.GridPos = Vector3i(gridX, gridY,gridZ);
//        });
//
//    //sort particles according to hash
//    std::sort(std::execution::par, m_particles.begin(), m_particles.end(), [](const Particle& a, const Particle& b) {
//        return a.Gridhash < b.Gridhash;
//        });
//
//    //create hash lookup for faster navigation
//    m_hashLookupTable[m_particles[0].Gridhash] = 0;
//    for (int i = 1; i < m_numParticles; i++) {
//        if (m_particles[i].Gridhash != m_particles[i - 1].Gridhash)
//        {
//            m_hashLookupTable[m_particles[i].Gridhash] = i;
//        }
//    }
//}
#pragma endregion
