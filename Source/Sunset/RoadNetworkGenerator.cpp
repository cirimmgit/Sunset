// Fill out your copyright notice in the Description page of Project Settings.


#include "RoadNetworkGenerator.h"
#include "DrawDebugHelpers.h"

// Sets default values
ARoadNetworkGenerator::ARoadNetworkGenerator()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
	SetRootComponent(SceneRoot);

}

// Called when the game starts or when spawned
void ARoadNetworkGenerator::BeginPlay()
{
	Super::BeginPlay();
}

// Minor Graph Ops

int32 ARoadNetworkGenerator::AddNode(const FVector& LocalPosition)
{
	const int32 VertexId =
		RoadNetwork.Graph.AppendVertex(LocalPosition);
	RoadNetwork.NodeData.Add(VertexId, FRoadNodeData());

	return VertexId;
}

int32 ARoadNetworkGenerator::AddEdge(int32 StartIndex, int32 EndIndex, ERoadType RoadType)
{

	if (StartIndex == EndIndex || (!RoadNetwork.Graph.IsVertex(StartIndex) || !RoadNetwork.Graph.IsVertex(EndIndex)))
	{
		return UE::Geometry::FDynamicGraph3d::InvalidID;
	}

	if (RoadNetwork.Graph.FindEdge(StartIndex, EndIndex) != UE::Geometry::FDynamicGraph3d::InvalidID)
	{
		return UE::Geometry::FDynamicGraph3d::InvalidID;
	}

	const int32 EdgeId = RoadNetwork.Graph.AppendEdge(StartIndex, EndIndex);

	FRoadEdgeData EdgeData;

	EdgeData.RoadType = RoadType;

	RoadNetwork.EdgeData.Add(EdgeId, EdgeData);


	return EdgeId;
}

bool ARoadNetworkGenerator::IsIntersectionAngleValid(const FRoadCandidate& Candidate, 
	const FRoadIntersectionResult& IntersectionResult
) const
{

	if (IntersectionResult.CandidateSegmentId >= Candidate.RoadPoints.Num() ||
		IntersectionResult.CandidateSegmentId + 1 >= Candidate.RoadPoints.Num() ||
		IntersectionResult.TargetEdgeId == UE::Geometry::FDynamicGraph::InvalidID)
	{
		return false;
	}

	const FVector CandidateSegmentStart = Candidate.RoadPoints[IntersectionResult.CandidateSegmentId];
	const FVector CandidateSegmentEnd = Candidate.RoadPoints[IntersectionResult.CandidateSegmentId+1];

	UE::Geometry::FDynamicGraph::FEdge TargetEdge = RoadNetwork.Graph.GetEdge(IntersectionResult.TargetEdgeId);

	const TArray<FVector> PolyLine = GetEdgePolyline(IntersectionResult.TargetEdgeId);

	if (IntersectionResult.TargetSegmentIndex >= PolyLine.Num() || IntersectionResult.TargetSegmentIndex + 1 >= PolyLine.Num())
	{
		return false;
	}

	const FVector TargetSegmentStart = PolyLine[IntersectionResult.TargetSegmentIndex];
	const FVector TargetSegmentEnd = PolyLine[IntersectionResult.TargetSegmentIndex+1];

	const FVector CandidateNormalizedDir = (CandidateSegmentEnd - CandidateSegmentStart).GetSafeNormal();
	const FVector TargetNormalizedDir = (TargetSegmentEnd - TargetSegmentStart).GetSafeNormal();

	if (CandidateNormalizedDir.IsNearlyZero() || TargetNormalizedDir.IsNearlyZero())
	{
		return false;
	}

	float DotProd = FMath::Clamp(
		FMath::Abs(
		FVector::DotProduct(CandidateNormalizedDir, TargetNormalizedDir)), 0 , 1);

	float AngleToDegrees = FMath::RadiansToDegrees(FMath::Acos(DotProd));
	
	return AngleToDegrees >= MinIntersectionAngle;

}

// Major Graph Ops

FNearestEdgeResult ARoadNetworkGenerator::FindNearestEdgeToNode(
	const FVector& LocalPosition,
	float MaxDistance,
	int32 IgnoreVertexId
) const
{
	FNearestEdgeResult BestResult;
	BestResult.Distance = MaxDistance;
	int32 BestVertexId = UE::Geometry::FDynamicGraph3d::InvalidID;

	for (int32 EdgeId : RoadNetwork.Graph.EdgeIndices())
	{
		const UE::Geometry::FDynamicGraph::FEdge CandidateEdge = RoadNetwork.Graph.GetEdge(EdgeId);

		if (CandidateEdge.A == IgnoreVertexId || CandidateEdge.B == IgnoreVertexId)
		{
			continue;
		}

		const TArray<FVector> EdgePoints = GetEdgePolyline(EdgeId);

		for (int32 SegmentIndex = 0; SegmentIndex < EdgePoints.Num() - 1; SegmentIndex++)
		{
			const FVector SegmentStart = EdgePoints[SegmentIndex];

			const FVector SegmentEnd = EdgePoints[SegmentIndex + 1];

			const FVector ClosestNodeOnSegment = FMath::ClosestPointOnSegment(LocalPosition, SegmentStart, SegmentEnd);

			float Distance = FVector::Dist(LocalPosition, ClosestNodeOnSegment);

			if (Distance < BestResult.Distance)
			{
				BestResult.bFound = true;
				BestResult.ClosestPoint = ClosestNodeOnSegment;
				BestResult.Distance = Distance;
				BestResult.EdgeId = EdgeId;
				BestResult.SegmentIndex = SegmentIndex;
			}
		}


	}

	return BestResult;
}

FRoadEdgeSplitResult ARoadNetworkGenerator::SplitRoadEdge(
	int32 EdgeId,
	int32 SegmentIndex,
	const FVector& SplitPos
)
{
	FRoadEdgeSplitResult SplitResult;


	if (!RoadNetwork.Graph.IsEdge(EdgeId))
	{
		return SplitResult;
	}

	TArray<FVector> OldPolyLine = GetEdgePolyline(EdgeId);

	if (SegmentIndex >= OldPolyLine.Num() -1 || SegmentIndex < 0)
	{
		return SplitResult;
	}

	const UE::Geometry::FDynamicGraph::FEdge OldEdge = RoadNetwork.Graph.GetEdge(EdgeId);

	const FRoadEdgeData* OldEdgeDataPtr = RoadNetwork.EdgeData.Find(EdgeId);

	if (!OldEdgeDataPtr)
	{
		return SplitResult;
	}

	const FRoadEdgeData OldEdgeData = *OldEdgeDataPtr;

	const FVector OldStartPos = RoadNetwork.Graph.GetVertex(OldEdge.A);
	const FVector OldEndPos = RoadNetwork.Graph.GetVertex(OldEdge.B);

	float EndpointTolerance = 1.0f;
	if (SplitPos.Equals(OldStartPos, EndpointTolerance))
	{
		SplitResult.bFound = true;
		SplitResult.SplitVertexId = OldEdge.A;
		return SplitResult;
	}

	if (SplitPos.Equals(OldEndPos, EndpointTolerance))
	{
		SplitResult.bFound = true;
		SplitResult.SplitVertexId = OldEdge.B;
		return SplitResult;
	}

	TArray<FVector> SplitPointsLhs;
	TArray<FVector> SplitPointsRhs;

	for (int SegmentItr = 1; SegmentItr <= SegmentIndex; SegmentItr++)
	{

		if (!OldPolyLine[SegmentItr].Equals(SplitPos, EndpointTolerance))
		{
			SplitPointsLhs.Add(OldPolyLine[SegmentItr]);
		}

	}

	for (int SegmentItr = SegmentIndex + 1; SegmentItr <= OldPolyLine.Num() - 1; SegmentItr++)
	{

		if (!OldPolyLine[SegmentItr].Equals(SplitPos, EndpointTolerance))
		{
			SplitPointsRhs.Add(OldPolyLine[SegmentItr]);
		}
	}

	int32 SplitNodeId = AddNode(SplitPos);

	if (SplitNodeId == UE::Geometry::FDynamicGraph::InvalidID)
	{
		return SplitResult;
	}

	UE::Geometry::FDynamicGraph::FEdgeSplitInfo SplitInfo;

	UE::Geometry::EMeshResult MeshSplitResult = RoadNetwork.Graph.SplitEdgeWithExistingVertex(EdgeId, SplitNodeId, SplitInfo);

	if (!(MeshSplitResult == UE::Geometry::EMeshResult::Ok))
	{
		RoadNetwork.Graph.RemoveVertex(SplitNodeId, true);
		FRoadNodeData* CleanupData = RoadNetwork.NodeData.Find(SplitNodeId);
		//delete CleanupData;
		RoadNetwork.NodeData.Remove(SplitNodeId);
		return SplitResult;
	}

	int32 NewEdgeLhsId = RoadNetwork.Graph.FindEdge(OldEdge.A, SplitNodeId);
	int32 NewEdgeRhsId = RoadNetwork.Graph.FindEdge(SplitNodeId, OldEdge.B);

	if (NewEdgeLhsId == UE::Geometry::FDynamicGraph::InvalidID || NewEdgeRhsId == UE::Geometry::FDynamicGraph::InvalidID)
	{
		return SplitResult;
	}

	FRoadEdgeData* CleanupEdgeData = RoadNetwork.EdgeData.Find(EdgeId);
	//delete CleanupEdgeData;
	RoadNetwork.EdgeData.Remove(EdgeId);

	UE::Geometry::FDynamicGraph::FEdge NewEdgeLhs = RoadNetwork.Graph.GetEdge(NewEdgeLhsId);

	UE::Geometry::FDynamicGraph::FEdge NewEdgeRhs = RoadNetwork.Graph.GetEdge(NewEdgeRhsId);

	FRoadEdgeData NewEdgeLhsData;
	
	NewEdgeLhsData.RoadType = OldEdgeData.RoadType;

	FRoadEdgeData NewEdgeRhsData;

	NewEdgeRhsData.RoadType = OldEdgeData.RoadType;

	OrientAndSetInteriorPoints(NewEdgeLhsId, OldEdge.A, SplitPointsLhs, NewEdgeLhsData);

	OrientAndSetInteriorPoints(NewEdgeRhsId, SplitNodeId, SplitPointsRhs, NewEdgeRhsData);

	RoadNetwork.EdgeData.Add(NewEdgeLhsId, NewEdgeLhsData);
	RoadNetwork.EdgeData.Add(NewEdgeRhsId, NewEdgeRhsData);

	SplitResult.bFound = true;

	SplitResult.FirstEdgeId = NewEdgeLhsId;
	SplitResult.SecondEdgeId = NewEdgeRhsId;
	SplitResult.SplitVertexId = SplitNodeId;

	return SplitResult;
}

FRoadIntersectionResult ARoadNetworkGenerator::FindFirstRoadIntersection(const FRoadCandidate& Candidate) const
{
	FRoadIntersectionResult BestResult;

	float CumulDistance = 0.0f;

	FVector CandidateStart = RoadNetwork.Graph.GetVertex(Candidate.StartVertexId);

	for (int32 SegmentId = 0; SegmentId < Candidate.RoadPoints.Num() - 1; SegmentId++)
	{
		const FVector SegmentStartPos = Candidate.RoadPoints[SegmentId];
		const FVector SegmentEndPos = Candidate.RoadPoints[SegmentId + 1];

		for (int32 EdgeId : RoadNetwork.Graph.EdgeIndices())
		{
			UE::Geometry::FDynamicGraph::FEdge Edge = RoadNetwork.Graph.GetEdge(EdgeId);


			if (Edge.A == Candidate.StartVertexId || Edge.B == Candidate.StartVertexId)
			{
				continue;
			}

			const TArray<FVector> GraphPolyline = GetEdgePolyline(EdgeId);

			for (int32 ExistingPolyLineIndex = 0; ExistingPolyLineIndex < GraphPolyline.Num() - 1; ExistingPolyLineIndex++)
			{
				FVector PolyLineSegmentStart = GraphPolyline[ExistingPolyLineIndex];
				FVector PolyLineSegmentEnd = GraphPolyline[ExistingPolyLineIndex + 1];

				FVector IntersectionPoint;
				bool IntersectionFound = FMath::SegmentIntersection2D(SegmentStartPos, SegmentEndPos, PolyLineSegmentStart, PolyLineSegmentEnd, IntersectionPoint);

				if (!IntersectionFound)
				{
					continue;
				}


				float DistanceToIntersection = CumulDistance + FVector::Dist(SegmentStartPos, IntersectionPoint);

				if (DistanceToIntersection < BestResult.DistanceAlongCandidate)
				{
					BestResult.bFound = true;
					BestResult.CandidateSegmentId = SegmentId;
					BestResult.DistanceAlongCandidate = DistanceToIntersection;
					BestResult.IntersectionPoint = IntersectionPoint;
					BestResult.TargetEdgeId = EdgeId;
					BestResult.TargetSegmentIndex = ExistingPolyLineIndex;
				}
			}
		}

		CumulDistance += FVector::Dist(SegmentStartPos, SegmentEndPos);
	}
	return BestResult;
}

// Helper Methods

FVector ARoadNetworkGenerator::DirectionFromDegrees(float AngleDegrees) const
{
	const float AngleRadians =
		FMath::DegreesToRadians(AngleDegrees);

	return FVector(
		FMath::Cos(AngleRadians),
		FMath::Sin(AngleRadians),
		0.0f
	);
}

// O(n) for now but will have to replace

int32 ARoadNetworkGenerator::FindNearestNode(
	const FVector& LocalPosition,
	float MaxDistance,
	int32 IgnoreVertexId) const
{
	float BestDistance = TNumericLimits<float>::Max();
	int32 BestNodeId = UE::Geometry::FDynamicGraph3d::InvalidID;

	for (int32 VertexIndex : RoadNetwork.Graph.VertexIndices())
	{
		FVector ComparisonVertex = RoadNetwork.Graph.GetVertex(VertexIndex);

		if (ComparisonVertex == LocalPosition || VertexIndex == IgnoreVertexId)
		{
			continue;
		}
		float Distance = FVector::Dist(LocalPosition, ComparisonVertex);

		if (Distance < MaxDistance && Distance < BestDistance)
		{
			BestDistance = Distance;
			BestNodeId = VertexIndex;
		}
	}
	return BestNodeId;
}

void ARoadNetworkGenerator::TruncateCandidateSegment(
	FRoadCandidate& Candidate,
	int32 SegmentIndex,
	const FVector& NewEndpoint
) const
{
	for (int32 i = Candidate.RoadPoints.Num() - 1; i > SegmentIndex; i--)
	{
		Candidate.RoadPoints.RemoveAt(i);
	}

	if (!Candidate.RoadPoints.Last().Equals(NewEndpoint, 1.0))
	{
		Candidate.RoadPoints.Add(NewEndpoint);
	}
}

TArray<FVector> ARoadNetworkGenerator::GetEdgePolyline(int32 EdgeId) const
{
	TArray<FVector> Points;

	if (!RoadNetwork.Graph.IsEdge(EdgeId))
	{
		return Points;
	}

	const FVector StartPos = RoadNetwork.Graph.GetVertex(RoadNetwork.Graph.GetEdge(EdgeId).A);
	const FVector EndPos = RoadNetwork.Graph.GetVertex(RoadNetwork.Graph.GetEdge(EdgeId).B);

	Points.Add(StartPos);

	const FRoadEdgeData* EdgeData = RoadNetwork.EdgeData.Find(EdgeId);

	if (EdgeData)
	{
		Points.Append(EdgeData->InteriorPoints);
	}

	Points.Add(EndPos);

	return Points;
}

void ARoadNetworkGenerator::OrientAndSetInteriorPoints(int32 EdgeId,
	int32 IntendedStartIndex,
	const TArray<FVector>& InteriorPoints,
	FRoadEdgeData& EdgeData) const
{
	UE::Geometry::FDynamicGraph::FEdge Edge = RoadNetwork.Graph.GetEdge(EdgeId);

	EdgeData.InteriorPoints = InteriorPoints;

	if (Edge.B == IntendedStartIndex)
	{
		Algo::Reverse(EdgeData.InteriorPoints);
	}
}

void ARoadNetworkGenerator::RefitCandidateToEndpoint(
	FRoadCandidate& Candidate,
	const FVector& NewEndPosition
) const
{
	if (Candidate.RoadPoints.Num() < 2)
	{
		return;
	}

	const FVector StartPosition =
		Candidate.RoadPoints[0];

	if (FMath::IsNearlyZero((NewEndPosition - StartPosition).Size()))
	{
		return;
	}

	Candidate.RoadPoints.Reset();

	Candidate.RoadPoints.Add(StartPosition);

	Candidate.RoadPoints.Add(NewEndPosition);
}

// Road Gen

FRoadCandidate ARoadNetworkGenerator::GenerateRoadCandidate(int32 StartingVertexId,
	const FVector& Direction,
	float Length,
	ERoadType RoadType)
{
	FRoadCandidate Candidate;

	if (!RoadNetwork.Graph.IsVertex(StartingVertexId))
	{
		return Candidate;
	}

	const FVector StartPos = RoadNetwork.Graph.GetVertex(StartingVertexId);

	const FVector NormalizedDir = Direction.GetSafeNormal();

	if (NormalizedDir.IsNearlyZero())
	{
		return Candidate;
	}

	Candidate.StartVertexId = StartingVertexId;

	Candidate.RoadType = RoadType;

	Candidate.RoadPoints.Add(StartPos);

	const FVector Perpendicular(
		-NormalizedDir.Y,
		NormalizedDir.X,
		0.0f
	);

	const FVector EndPos = StartPos + (Length * NormalizedDir);

	Candidate.RoadPoints.Add(EndPos);

	return Candidate;

}

FRoadConstraintResult ARoadNetworkGenerator::ApplyLocalConstraints(FRoadCandidate& Candidate) const
{
	FRoadConstraintResult Result;

	if (!Candidate.isValid())
	{
		return Result;
	}

	if (!RoadNetwork.Graph.IsVertex(Candidate.StartVertexId))
	{
		return Result;
	}

	const FVector& CurrentLastPos = Candidate.RoadPoints.Last();

	// Check intersection (prio over node)

	const FRoadIntersectionResult IntersectionResult = FindFirstRoadIntersection(Candidate);

	if (IntersectionResult.bFound)
	{
		if (!IsIntersectionAngleValid(Candidate, IntersectionResult))
		{
			return Result;
		}
		TruncateCandidateSegment(Candidate, IntersectionResult.CandidateSegmentId, IntersectionResult.IntersectionPoint);

		Result.bAccepted = true;
		Result.ConnectionTarget = ERoadConnectionTarget::ExistingEdge;
		Result.TargetEdgeId = IntersectionResult.TargetEdgeId;
		Result.TargetSegmentIndex = IntersectionResult.TargetSegmentIndex;
		Result.TargetPoint = IntersectionResult.IntersectionPoint;
		return Result;
	}

	// Check node snap (prioritized over edges)
	const int32 ProposedLastId = FindNearestNode(CurrentLastPos, NodeSnapDistance, Candidate.StartVertexId);

	if (ProposedLastId != UE::Geometry::FDynamicGraph::InvalidID)
	{		
		const FVector SnapPos = RoadNetwork.Graph.GetVertex(ProposedLastId);

		RefitCandidateToEndpoint(Candidate, SnapPos);
		// Connect to existing vertex during commit
		Result.ConnectionTarget = ERoadConnectionTarget::ExistingVertex;

		Result.TargetVertexId = ProposedLastId;
	}
	else
	{
		// Prioritize edges over nodes
		FNearestEdgeResult ProposedNearestEdge = FindNearestEdgeToNode(CurrentLastPos, EdgeSnapDistance, Candidate.StartVertexId);

		if (ProposedNearestEdge.bFound)
		{
			const FVector SnapPos = ProposedNearestEdge.ClosestPoint;

			RefitCandidateToEndpoint(Candidate, SnapPos);

			Result.ConnectionTarget = ERoadConnectionTarget::ExistingEdge;

			Result.TargetEdgeId = ProposedNearestEdge.EdgeId;
			
			Result.TargetSegmentIndex = ProposedNearestEdge.SegmentIndex;
			
			Result.TargetPoint = SnapPos;
		}
	}

	Result.bAccepted = true;

	return Result;

}

int32 ARoadNetworkGenerator::CommitRoadCandidate(
	const FRoadCandidate& Candidate,
	const FRoadConstraintResult& RoadConstraint
)
{
	const FVector StartPos = Candidate.RoadPoints[0];

	if (!RoadNetwork.Graph.IsVertex(Candidate.StartVertexId) || !Candidate.isValid())
	{
		return UE::Geometry::FDynamicGraph::InvalidID;
	}

	int32 EndVertexId = UE::Geometry::FDynamicGraph::InvalidID;

	// Gotta change the endpoint
	if (RoadConstraint.ConnectionTarget == ERoadConnectionTarget::ExistingVertex)
	{	
		EndVertexId = RoadConstraint.TargetVertexId;
	}
	else if (RoadConstraint.ConnectionTarget == ERoadConnectionTarget::ExistingEdge)
	{
		FRoadEdgeSplitResult EdgeResult = SplitRoadEdge(RoadConstraint.TargetEdgeId, RoadConstraint.TargetSegmentIndex, RoadConstraint.TargetPoint);

		if (!EdgeResult.bFound)
		{
			return UE::Geometry::FDynamicGraph::InvalidID;
		}
		EndVertexId = EdgeResult.SplitVertexId;
	}
	else 
	{
		const FVector& EndPos = Candidate.RoadPoints.Last();

		EndVertexId = AddNode(EndPos);
	}

	if (
		EndVertexId ==
		UE::Geometry::FDynamicGraph::InvalidID || EndVertexId == Candidate.StartVertexId)
	{
		return UE::Geometry::FDynamicGraph::InvalidID;
	}

	int32 EdgeId = AddEdge(Candidate.StartVertexId, EndVertexId, Candidate.RoadType);

	if (!RoadNetwork.Graph.IsEdge(EdgeId))
	{
		// TODO: Cleanup orphan node
		return UE::Geometry::FDynamicGraph3d::InvalidID;
	}

	FRoadEdgeData* EdgeData = RoadNetwork.EdgeData.Find(EdgeId);

	if (!EdgeData)
	{
		// TODO: Cleanup orphan node
		return UE::Geometry::FDynamicGraph3d::InvalidID;
	}

	if (RoadNetwork.Graph.GetEdge(EdgeId).A == Candidate.StartVertexId)
	{
		// Weird undirected stuff, can be in A -> B format after snapping changes
		for (
			int32 InteriorPoint = 1;
			InteriorPoint < Candidate.RoadPoints.Num() - 1;
			++InteriorPoint
			)
		{
			EdgeData->InteriorPoints.Add(
				Candidate.RoadPoints[InteriorPoint]
			);
		}
	}
	else if (RoadNetwork.Graph.GetEdge(EdgeId).B == Candidate.StartVertexId)
	{
		// Reverse inner points if not in A -> B order (B -> A)
		for (
			int32 InteriorPoint =
			Candidate.RoadPoints.Num() - 2;
			InteriorPoint >= 1;
			--InteriorPoint
			)
		{
			EdgeData->InteriorPoints.Add(
				Candidate.RoadPoints[InteriorPoint]
			);
		}
	}
	else
	{
		// Cross fingers and hope we dont end up here somehow
		return UE::Geometry::FDynamicGraph::InvalidID;
	}

	return EndVertexId;
}

void ARoadNetworkGenerator::DrawRoadGraph()
{

	if (!GetWorld())
	{
		return;
	}

	FlushPersistentDebugLines(GetWorld());

	const FTransform ActorTransform = GetActorTransform();


	for (int32 VertexId : RoadNetwork.Graph.VertexIndices())
	{

		const FVector LocalPosition = RoadNetwork.Graph.GetVertex(VertexId);

		const FVector NodePos = ActorTransform.TransformPosition(LocalPosition);

		DrawDebugSphere(
			GetWorld(),
			NodePos,
			NodeRadius,
			16,
			FColor::Red,
			true
		);
	}

	for (int32 EdgeId : RoadNetwork.Graph.EdgeIndices())
	{

		FColor EdgeColor = FColor::Green;
		float Thickness = 10.0f;

		FRoadEdgeData* EdgeData = RoadNetwork.EdgeData.Find(EdgeId);

		if (EdgeData)
		{
			switch (EdgeData->RoadType)
			{
			case ERoadType::Arterial:
				EdgeColor = FColor::Yellow;
				Thickness = 30.0f;
				break;
			case ERoadType::Collector:
				EdgeColor = FColor::Cyan;
				Thickness = 20.0f;
				break;
			case ERoadType::Local:
				EdgeColor = FColor::Green;
				Thickness = 10.0f;
				break;
			}
		}
		else {
			continue;
		}

		const TArray<FVector> EdgePoints = GetEdgePolyline(EdgeId);



		for (int32 CurrNodeIndex = 0; CurrNodeIndex < EdgePoints.Num() - 1; CurrNodeIndex++)
		{

			const FVector CurrNodeTransformed = ActorTransform.TransformPosition(EdgePoints[CurrNodeIndex]);
			const FVector NextNodeTransformed = ActorTransform.TransformPosition(EdgePoints[CurrNodeIndex + 1]);

			DrawDebugLine(
				GetWorld(),
				CurrNodeTransformed,
				NextNodeTransformed,
				EdgeColor,
				true,
				-1.0f,
				0,
				Thickness
			);
		}
	}
}

void ARoadNetworkGenerator::GenerateRoadNetwork()
{
	RoadNetwork.Reset();

	FRandomStream RandomStream(Seed);

	TArray<int32> ArterialNodes;

	int32 CurrNodeId = AddNode(GetActorLocation());

	ArterialNodes.Add(CurrNodeId);

	float CurrHeading = 0.0f;

	for (int32 i = 0; i < ArterialSegmentCount; i++)
	{
		CurrHeading += RandomStream.FRandRange(-8.0f, 8.0f);

		FRoadCandidate ArterialCandidate = GenerateRoadCandidate(CurrNodeId, DirectionFromDegrees(CurrHeading), RoadLength, ERoadType::Arterial);

		const FRoadConstraintResult ArterialConstraintResult = ApplyLocalConstraints(ArterialCandidate);

		if (!ArterialConstraintResult.bAccepted)
		{
			return;
		}

		int32 NewNodeId = CommitRoadCandidate(ArterialCandidate, ArterialConstraintResult);

		if (NewNodeId == UE::Geometry::FDynamicGraph3d::InvalidID)
		{
			break;
		}

		ArterialNodes.Add(NewNodeId);

		CurrNodeId = NewNodeId;
	}

	for (int32 i = 1; i < ArterialSegmentCount - 1; i++)
	{
		const int32 ArterialNodeId = ArterialNodes[i];

		const int32 NextArterialNodeId = ArterialNodes[i + 1];

		const FVector ArterialNodePos = RoadNetwork.Graph.GetVertex(ArterialNodeId);

		const FVector NextArterialNodePos = RoadNetwork.Graph.GetVertex(NextArterialNodeId);

		const FVector ArterialDirection = (NextArterialNodePos - ArterialNodePos).GetSafeNormal();

		const float ArterialHeading = FMath::RadiansToDegrees(FMath::Atan2(ArterialDirection.Y, ArterialDirection.X));

		for (int32 Side : { -1, 1})
		{
			const float CollectorLength = RoadLength * 0.75f;

			UE_LOG(
				LogTemp,
				Warning,
				TEXT(
					"Arterial %d, Side %d: Heading=%.2f Offset=%.2f Final=%.2f"
				),
				i,
				Side,
				ArterialHeading,
				BranchAngle,
				ArterialHeading + Side * BranchAngle
			);

			const float CollectorAngle = ArterialHeading + Side * RandomStream.FRandRange(35.0f, 55.0f);

			const FVector CollectorDirection = DirectionFromDegrees(CollectorAngle);

			FRoadCandidate CollectorCandidate = GenerateRoadCandidate(ArterialNodeId, CollectorDirection, CollectorLength, ERoadType::Collector);

			const FRoadConstraintResult CollectorCandidateResult = ApplyLocalConstraints(CollectorCandidate);

			if (!CollectorCandidateResult.bAccepted)
			{
				return;
			}

			int32 NewCollectorNodeId = CommitRoadCandidate(CollectorCandidate, CollectorCandidateResult);

			if (NewCollectorNodeId == UE::Geometry::FDynamicGraph3d::InvalidID)
			{
				continue;
			}

			for (int32 LocalSide : { -1, 1})
			{
				const float LocalLength = RoadLength * 0.40f;

				const float LocalAngle = CollectorAngle + LocalSide * RandomStream.FRandRange(35.0f, 55.0f);

				const FVector LocalDirection = DirectionFromDegrees(LocalAngle);

				FRoadCandidate LocalCandidate = GenerateRoadCandidate(NewCollectorNodeId, LocalDirection, LocalLength, ERoadType::Local);

				const FRoadConstraintResult LocalCandidateResult = ApplyLocalConstraints(LocalCandidate);

				if (!LocalCandidateResult.bAccepted)
				{
					return;
				}

				int32 NewLocalNodeId = CommitRoadCandidate(LocalCandidate, LocalCandidateResult);

				if (NewLocalNodeId == UE::Geometry::FDynamicGraph3d::InvalidID)
				{
					continue;
				}

			}

		}

	}

	DrawRoadGraph();

}


// Test Methods


int32 ARoadNetworkGenerator::AddTestRoad(
	const FVector& StartPos,
	const FVector& EndPos,
	ERoadType RoadType,
	const TArray<FVector>& InteriorPoints
)
{
	const int32 StartVertexId = AddNode(StartPos);
	const int32 EndVertexId = AddNode(EndPos);

	const int32 EdgeId = AddEdge(StartVertexId, EndVertexId, RoadType);

	if (!RoadNetwork.Graph.IsEdge(EdgeId))
	{
		return UE::Geometry::FDynamicGraph::InvalidID;
	}

	FRoadEdgeData* EdgeData = RoadNetwork.EdgeData.Find(EdgeId);

	if (!EdgeData)
	{
		return UE::Geometry::FDynamicGraph::InvalidID;
	}

	EdgeData->InteriorPoints = InteriorPoints;
	return EdgeId;
}


void ARoadNetworkGenerator::Test_Clear()
{
	RoadNetwork.Reset();
	if (!GetWorld())
	{
		return;
	}

	FlushPersistentDebugLines(GetWorld());
}

void ARoadNetworkGenerator::Test_NodeSnapping()
{

	

	int32 StartNodeId = AddNode(FVector(0, 0, 0));
	int32 EndNodeId = AddNode(FVector(10000, 2000, 0));

	FRoadCandidate Candidate = GenerateRoadCandidate(StartNodeId, FVector(1,0,0), 10000.0f, ERoadType::Arterial);

	const FVector EndPos = Candidate.RoadPoints.Last();

	FRoadConstraintResult ConstraintResult = ApplyLocalConstraints(Candidate);

	if (ConstraintResult.bAccepted)
	{
		CommitRoadCandidate(Candidate, ConstraintResult);
	}

	const FVector WorldOriginalEnd =
		GetActorTransform().TransformPosition(
			EndPos
		);

	DrawRoadGraph();

	// show original proposed endpoint
	DrawDebugSphere(
		GetWorld(),
		WorldOriginalEnd,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);

}

void ARoadNetworkGenerator::Test_EdgeSnapping()
{
	int32 ExistingRoadId = AddTestRoad(FVector(0, 0, 0), FVector(10000, 0, 0), ERoadType::Arterial);

	int32 NewStartNodeId = AddNode(FVector(5000, -5000, 0));

	FRoadCandidate Candidate = GenerateRoadCandidate(NewStartNodeId, FVector(0, 1, 0), 4000.0f, ERoadType::Collector);

	const FVector EndPos = Candidate.RoadPoints.Last();


	FRoadConstraintResult ConstraintResult = ApplyLocalConstraints(Candidate);

	if (ConstraintResult.bAccepted)
	{
		CommitRoadCandidate(Candidate, ConstraintResult);
	}

	const FVector WorldOriginalEnd =
		GetActorTransform().TransformPosition(
			EndPos
		);

	DrawRoadGraph();

	// show original proposed endpoint
	DrawDebugSphere(
		GetWorld(),
		WorldOriginalEnd,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);

}

void ARoadNetworkGenerator::Test_IntersectionAngleCheck()
{
	RoadNetwork.Reset();

	int32 Offset = 0;

	// Case 1: 90 deg

	int32 ExistingRoadId_1 = AddTestRoad(
		FVector(Offset + 0, 0, 0),
		FVector(Offset + 10000, 0, 0),
		ERoadType::Arterial
	);

	int32 NewStartNodeId_1 = AddNode(
		FVector(Offset + 5000, -5000, 0)
	);

	FRoadCandidate Candidate_1 = GenerateRoadCandidate(
		NewStartNodeId_1,
		DirectionFromDegrees(90.0f),
		10000.0f,
		ERoadType::Collector
	);

	const FVector EndPos_1 =
		Candidate_1.RoadPoints.Last();

	FRoadConstraintResult ConstraintResult_1 =
		ApplyLocalConstraints(Candidate_1);

	UE_LOG(
		LogTemp,
		Warning,
		TEXT("90 degree accepted: %s"),
		ConstraintResult_1.bAccepted
		? TEXT("true")
		: TEXT("false")
	);

	if (ConstraintResult_1.bAccepted)
	{
		CommitRoadCandidate(
			Candidate_1,
			ConstraintResult_1
		);
	}

	const FVector WorldOriginalEnd_1 =
		GetActorTransform().TransformPosition(
			EndPos_1
		);


	// Case 2: 45 deg

	Offset = 12000;

	int32 ExistingRoadId_2 = AddTestRoad(
		FVector(Offset + 0, 0, 0),
		FVector(Offset + 10000, 0, 0),
		ERoadType::Arterial
	);

	int32 NewStartNodeId_2 = AddNode(
		FVector(Offset + 2500, -2500, 0)
	);

	FRoadCandidate Candidate_2 = GenerateRoadCandidate(
		NewStartNodeId_2,
		DirectionFromDegrees(45.0f),
		8000.0f,
		ERoadType::Collector
	);

	const FVector EndPos_2 =
		Candidate_2.RoadPoints.Last();

	FRoadConstraintResult ConstraintResult_2 =
		ApplyLocalConstraints(Candidate_2);

	UE_LOG(
		LogTemp,
		Warning,
		TEXT("45 degree accepted: %s"),
		ConstraintResult_2.bAccepted
		? TEXT("true")
		: TEXT("false")
	);

	if (ConstraintResult_2.bAccepted)
	{
		CommitRoadCandidate(
			Candidate_2,
			ConstraintResult_2
		);
	}

	const FVector WorldOriginalEnd_2 =
		GetActorTransform().TransformPosition(
			EndPos_2
		);


	// Case 3: 15 deg

	Offset = 24000;

	int32 ExistingRoadId_3 = AddTestRoad(
		FVector(Offset + 0, 0, 0),
		FVector(Offset + 10000, 0, 0),
		ERoadType::Arterial
	);

	int32 NewStartNodeId_3 = AddNode(
		FVector(Offset + 1000, -1000, 0)
	);

	FRoadCandidate Candidate_3 = GenerateRoadCandidate(
		NewStartNodeId_3,
		DirectionFromDegrees(15.0f),
		10000.0f,
		ERoadType::Collector
	);

	const FVector EndPos_3 =
		Candidate_3.RoadPoints.Last();

	FRoadConstraintResult ConstraintResult_3 =
		ApplyLocalConstraints(Candidate_3);

	UE_LOG(
		LogTemp,
		Warning,
		TEXT("15 degree accepted: %s"),
		ConstraintResult_3.bAccepted
		? TEXT("true")
		: TEXT("false")
	);

	if (ConstraintResult_3.bAccepted)
	{
		CommitRoadCandidate(
			Candidate_3,
			ConstraintResult_3
		);
	}

	const FVector WorldOriginalEnd_3 =
		GetActorTransform().TransformPosition(
			EndPos_3
		);

	DrawRoadGraph();

	DrawDebugSphere(
		GetWorld(),
		WorldOriginalEnd_1,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);

	DrawDebugSphere(
		GetWorld(),
		WorldOriginalEnd_2,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);

	DrawDebugSphere(
		GetWorld(),
		WorldOriginalEnd_3,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);
}

void ARoadNetworkGenerator::Test_CurvedEdgeSnapping()
{

	const TArray<FVector> InteriorCurveNode = { FVector(5000, -3000, 0), FVector(10000, -5000, 0), FVector(15000, -3000, 0) };

	int32 ExistingRoadId = AddTestRoad(FVector(0, 0, 0), FVector(20000, 0, 0), ERoadType::Arterial, InteriorCurveNode);

	int32 NewStartNodeId = AddNode(FVector(13000, -10000, 0));

	FRoadCandidate Candidate = GenerateRoadCandidate(NewStartNodeId, FVector(0, 1, 0), 4000.0f, ERoadType::Collector);

	const FVector EndPos = Candidate.RoadPoints.Last();

	FNearestEdgeResult SnapToEdgeCandidate = FindNearestEdgeToNode(EndPos, EdgeSnapDistance, NewStartNodeId);


	FRoadConstraintResult ConstraintResult = ApplyLocalConstraints(Candidate);

	if (ConstraintResult.bAccepted)
	{
		CommitRoadCandidate(Candidate, ConstraintResult);
	}

	const FVector TransformedOriginalEndPos =
		GetActorTransform().TransformPosition(
			EndPos
		);

	DrawRoadGraph();

	// show original proposed endpoint
	DrawDebugSphere(
		GetWorld(),
		TransformedOriginalEndPos,
		NodeRadius * 0.75f,
		12,
		FColor::Magenta,
		true
	);

	if (SnapToEdgeCandidate.bFound)
	{

		const FVector TransformedSnapToEdgePos =
			GetActorTransform().TransformPosition(
				SnapToEdgeCandidate.ClosestPoint
			);

		// show edge snap candidate
		DrawDebugSphere(
			GetWorld(),
			TransformedSnapToEdgePos,
			NodeRadius * 0.75f,
			12,
			FColor::Blue,
			true
		);
	}

}