#include "Agent.h"
#include "Chaos/Core.h"
#include "Engine/Engine.h"

AAgent::AAgent(){
	PrimaryActorTick.bCanEverTick = true;
	Velocity = FVector(0.f);

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("AgentMesh"));	
	RootComponent = Mesh;
}

void AAgent::BeginPlay(){
	Super::BeginPlay();
}

void AAgent::Init( UStaticMeshComponent *mesh, int id ) {
	UE_LOG(LogTemp, Warning, TEXT("Agent initialized.") );
	Mesh->SetStaticMesh( mesh->GetStaticMesh() );
}

void AAgent::Tick(float DeltaTime){
	Super::Tick(DeltaTime);

	FVector loc = GetActorLocation();
	SetActorLocation( loc + Velocity * DeltaTime );

	SetActorRotation(Velocity.GetSafeNormal().Rotation().Add(-90.f, 0.f, 0.f));
}
