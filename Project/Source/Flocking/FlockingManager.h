#pragma once

#include "CoreMinimal.h"
#include "FlockingManager.generated.h"

UCLASS()
class FLOCKING_API UFlockingManager : public UObject
{

public:
	GENERATED_BODY()

	void Init( UWorld *world, UStaticMeshComponent *mesh);
	void Flock();

private:
	UWorld *World;	
	bool initialized;
	TArray<class AAgent *> Agents;

	FVector ruleOne(AAgent* focusBoid);
	FVector ruleTwo(AAgent* focusBoid);
	FVector ruleThree(AAgent* focusBoid);
	FVector bound(AAgent* focusBoid);
};