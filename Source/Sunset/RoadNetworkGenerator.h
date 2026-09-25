// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Curve/DynamicGraph3.h"
#include "RoadNetworkGenerator.generated.h"

UENUM(BlueprintType)
enum class ERoadType : uint8
{
	Arterial UMETA(DisplayName = "Arterial"),
	Collector UMETA(DisplayName = "Collector"),
	Local UMETA(DisplayName = "Local")
};


USTRUCT(BlueprintType)
struct FRoadNodeData
{
	GENERATED_BODY()

};

USTRUCT(BlueprintType)
struct FRoadEdgeData
{
	GENERATED_BODY();


	UPROPERTY(VisibleAnywhere)
	ERoadType RoadType = ERoadType::Local;

	UPROPERTY(VisibleAnywhere)
	TArray<FVector> InteriorPoints;
};

struct FRoadNetwork
{
	UE::Geometry::FDynamicGraph3d Graph;

	TMap<int32, FRoadNodeData> NodeData;

	TMap<int32, FRoadEdgeData> EdgeData;

	void Reset()
	{
		NodeData.Empty();
		EdgeData.Empty();
		Graph = UE::Geometry::FDynamicGraph3d();
	}

};

struct FRoadCandidate
{
	int32 StartVertexId = UE::Geometry::FDynamicGraph::InvalidID;

	ERoadType RoadType = ERoadType::Local;

	//First point is start of road, last point end of road
	TArray<FVector> RoadPoints;

	bool isValid() const
	{
		return StartVertexId != UE::Geometry::FDynamicGraph::InvalidID &&
			RoadPoints.Num() >= 2;
	}

};


/// <summary>
/// Road Patches
/// </summary>

enum class EGenerationRoadClass
{
	Arterial,
	Local
};


// Small network representing patch of nodes/edges from example data
struct FRoadPatch
{
	UE::Geometry::FDynamicGraph3d Graph;

	int32 RootVertexId = UE::Geometry::FDynamicGraph::InvalidID;

	TArray<int32> ConnectorEdgeIds;

	// Frequency from example data

	double Weight = -1.0;
};


// Buncha seeds that the road network will grow from
struct FRoadGrowthSeed
{
	int32 VertexId;

	FVector PreferredDirection;

	EGenerationRoadClass RoadClass;

	int32 Depth = 0;
};

struct FLocalStreetSeed
{
	int32 ArterialEdgeId;

	double DistanceAlongEdge;

	FVector InitialDirection;

	int32 SourcePatchId;
};

struct FRoadLegalityResult
{
	bool bValid = true;

	bool bHitsWater = false;
	bool bTooSteep = false;
	bool bIntersectsRoad = false;
};

struct FNearestEdgeResult
{
	bool bFound = false;

	FVector ClosestPoint = FVector::ZeroVector;

	int32 EdgeId = UE::Geometry::FDynamicGraph::InvalidID;
	int32 SegmentIndex = INDEX_NONE;

	double Distance = TNumericLimits<double>::Max();
};

struct FRoadEdgeSplitResult
{
	bool bFound = false;

	int32 SplitVertexId = UE::Geometry::FDynamicGraph::InvalidID;

	int32 FirstEdgeId = UE::Geometry::FDynamicGraph::InvalidID;

	int32 SecondEdgeId = UE::Geometry::FDynamicGraph::InvalidID;
};

struct FRoadIntersectionResult
{
	bool bFound = false;

	int32 CandidateSegmentId = INDEX_NONE;

	int32 TargetEdgeId = UE::Geometry::FDynamicGraph::InvalidID;

	int32 TargetSegmentIndex = INDEX_NONE;

	FVector IntersectionPoint = FVector::ZeroVector;
	
	double DistanceAlongCandidate = TNumericLimits<float>::Max();

};

enum class ERoadConnectionTarget
{
	NewVertex,
	ExistingVertex,
	ExistingEdge
};

struct FRoadConstraintResult
{
	bool bAccepted = false;

	ERoadConnectionTarget ConnectionTarget = ERoadConnectionTarget::NewVertex;


	int32 TargetVertexId = UE::Geometry::FDynamicGraph::InvalidID;
	int32 TargetEdgeId = UE::Geometry::FDynamicGraph::InvalidID;
	int32 TargetSegmentIndex = INDEX_NONE;
	FVector TargetPoint = FVector::ZeroVector;
	
};

struct FRoadStyleStatistics
{
	double MeanLength = 10000.0;
	double LengthVariance = 1000.0;
	
	double MeanCurvature = 0.0;
	double CurvatureVariance = 0.0;

	double MeanBranchAngle = 90.0;
	double BranchAngleVariance = 10.0;
};

struct FRoadExample
{
	FRoadNetwork Network;

	TArray<FRoadPatch> ArterialPatches;
	TArray<FRoadPatch> LocalPatches;

	FRoadStyleStatistics ArterialStats;
	FRoadStyleStatistics LocalStats;

};



UCLASS()
class SUNSET_API ARoadNetworkGenerator : public AActor
{
	GENERATED_BODY()
	
public:	
	ARoadNetworkGenerator();

protected:
	virtual void BeginPlay() override;

public: 

	// Components

	UPROPERTY(VisibleAnywhere, Category = "Components")
	USceneComponent* SceneRoot;

	// Gen settings

	UPROPERTY(EditAnywhere, Category = "Road Generation")
	int32 Seed = 1;

	UPROPERTY(EditAnywhere, Category = "Road Generation", meta = (ClampMin = "1"))
	int32 ArterialSegmentCount = 5;

	UPROPERTY(EditAnywhere, Category = "Road Generation")
	float RoadLength = 10000.0f;

	UPROPERTY(EditAnywhere, Category = "Road Generation")
	float BranchAngle = 35.0f;

	UPROPERTY(EditAnywhere, Category = "Road Generation")
	float NodeRadius = 100.0f;


	// Connection Settings

	UPROPERTY(EditAnywhere, Category = "Road Generation|Connections")
	float NodeSnapDistance = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Road Generation|Connections")
	float EdgeSnapDistance = 1000.0f;

	UPROPERTY(EditAnywhere, Category = "Road Generation|Connections")
	float MinIntersectionAngle = 25.0f;

	UFUNCTION(CallInEditor, Category = "Road Generation")
	void GenerateRoadNetwork();

	// In-Editor Tests

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_NodeSnapping();

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_EdgeSnapping();

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_CurvedEdgeSnapping();

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_Clear();

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_IntersectionSnapping();

	UFUNCTION(CallInEditor, Category = "Road Generation|Tests")
	void Test_IntersectionAngleCheck();

private:
	
	FRoadNetwork RoadNetwork;


	// Graph Mutate Ops
	int32 AddNode(const FVector& LocalPosition);

	int32 AddEdge(
		int32 StartVertexId,
		int32 EndVertexId,
		ERoadType RoadType
	);

	// Helpers
	int32 FindNearestNode(
		const FVector& LocalPosition,
		float MaxDistance,
		int32 IgnoreVertexId =
		UE::Geometry::FDynamicGraph::InvalidID
	) const;

	FNearestEdgeResult FindNearestEdgeToNode(
		const FVector& LocalPosition,
		float MaxDistance,
		int32 IgnoreVertexId =
		UE::Geometry::FDynamicGraph::InvalidID
	) const;

	FRoadEdgeSplitResult SplitRoadEdge(
		int32 EdgeId,
		int32 SegmentIndex,
		const FVector& SplitPos
	);

	void OrientAndSetInteriorPoints(
		int32 EdgeId,
		int32 IntendedStartIndex,
		const TArray<FVector>& InteriorPoints,
		FRoadEdgeData& EdgeData
	) const;

	void TruncateCandidateSegment(
		FRoadCandidate& Candidate,
		int32 SegmentIndex,
		const FVector& NewEndpoint) const;

	FVector DirectionFromDegrees(float AngleDegrees) const;

	bool IsIntersectionAngleValid(const FRoadCandidate& Candidate, const FRoadIntersectionResult& IntersectionResult) const;

	FRoadIntersectionResult FindFirstRoadIntersection(const FRoadCandidate& Candidate) const;

	//// RoadGen
	//double ScoreConnectorMatch(
	//	const FLineSegment& Existing,
	//	const FLineSegment& Candidate
	//) const;

	//FRoadLegalityResult CheckPatchLegality(
	//	const FTransformedRoadPatch& Patch
	//) const;

	bool TryProceduralGrowth(
		const FRoadGrowthSeed& Seed,
		const FRoadStyleStatistics& Style
	);

	//FLocalConstraintResult ApplyLocalConstraints(
	//	const FRoadCandidate& Candidate
	//);

	FRoadCandidate GenerateRoadCandidate(
		int32 StartingVertexId,
		const FVector& Direction,
		float Length,
		ERoadType RoadType
	);

	void RefitCandidateToEndpoint(
		FRoadCandidate& Candidate,
		const FVector& NewEndPosition
	) const;

	FRoadConstraintResult ApplyLocalConstraints(
		FRoadCandidate& Candidate
	) const;

	int32 CommitRoadCandidate(
		const FRoadCandidate& Candidate,
		const FRoadConstraintResult& Constraint
	);

	TArray<FVector> GetEdgePolyline(
		int32 EdgeId
	) const;

	int32 AddTestRoad(
		const FVector& StartPos,
		const FVector& EndPos,
		ERoadType RoadType,
		const TArray<FVector>& InteriorPoints = {}
	);

	void DrawRoadGraph();

};
