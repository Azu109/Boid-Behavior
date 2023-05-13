#include "FlockingManager.h"
#include "Agent.h"

#define AGENT_COUNT 20
#define MAX_SPEED 2000.f
#define BOUNDARY_RADIUS 2000

void UFlockingManager::Init( UWorld *world, UStaticMeshComponent *mesh) {
    UE_LOG(LogTemp, Warning, TEXT("Manager initialized"));

    
    World = world;
    if( World != nullptr ) {

        // Point generation algorithm adapted from:
        // http://extremelearning.com.au/evenly-distributing-points-on-a-sphere/
            
        for( int i = 0; i < AGENT_COUNT; i++ ) {
            FVector2D flatPoint = FMath::RandPointInCircle(BOUNDARY_RADIUS);

            FVector location = FVector(flatPoint, 0.f);

            AAgent* agent = World->SpawnActor<AAgent>(location, FRotator());
            agent->Init(mesh, i);
            Agents.Add(agent);
        }
    }

    initialized = true;
}

void UFlockingManager::Flock() {
// Pseudo code retrieved from http://www.kfish.org/boids/pseudocode.html

		for(auto boid : Agents) {
			auto v1 = ruleOne(boid);
			auto v2 = ruleTwo(boid);
			auto v3 = ruleThree(boid);
      auto v4 = bound(boid);

			boid->Velocity += v1 + v2 + v3 + v4 ;
      boid->Velocity = boid->Velocity.GetClampedToMaxSize(MAX_SPEED);
      boid->Velocity.Z = 0.f;
    }
}

// Rule 1: Boids try to fly towards the centre of mass of neighbouring boids.
FVector UFlockingManager::ruleOne(AAgent* focusBoid) {
    FVector centersTotal;

    for(auto boid : Agents) {
      if(boid != focusBoid) {
        centersTotal += boid->GetActorLocation();
      }
    }

    FVector averageCenter = centersTotal / (AGENT_COUNT-1);

    return (averageCenter - focusBoid->GetActorLocation()) / 50;
}

// Rule 2: Boids try to keep a small distance away from other objects (including other boids).
FVector UFlockingManager::ruleTwo(AAgent* focusBoid) {
    FVector vec;

    for(auto boid : Agents) {
      float distance = FVector::Distance(boid->GetActorLocation(), focusBoid->GetActorLocation());

      int repulsiveRadius = 500;
      if((boid != focusBoid) && (distance < repulsiveRadius)) {
        vec -= (boid->GetActorLocation() - focusBoid->GetActorLocation()) * repulsiveRadius * repulsiveRadius / (distance * distance * 10);
      }
    }

    return vec;
}

// Rule 3: Boids try to match velocity with near boids.
FVector UFlockingManager::ruleThree(AAgent* focusBoid) {
    FVector velocitiesTotal;

    for(auto boid : Agents) {
      if(boid != focusBoid) {
        velocitiesTotal += boid->Velocity;
      }
    }

    FVector averageVelocity = velocitiesTotal / (AGENT_COUNT-1);

    return (averageVelocity - focusBoid->Velocity) / 8;
}

FVector UFlockingManager::bound(AAgent* focusBoid) {
    FVector vec;
    
    FVector location = focusBoid->GetActorLocation();

    if(location.Size() > BOUNDARY_RADIUS) {
      vec = (location.GetSafeNormal() * BOUNDARY_RADIUS) - location;
    }

    return vec;
}
