using Autodesk.AutoCAD.Runtime;
using Autodesk.AutoCAD.ApplicationServices;
using Autodesk.AutoCAD.DatabaseServices;
using Autodesk.AutoCAD.EditorInput;
using Autodesk.AutoCAD.Geometry;
using Autodesk.AutoCAD.Colors;

using System;
using System.Linq;
using System.Collections.Generic;
namespace PluginCommands
{
    public class PluginCommands
    {
        [CommandMethod("PS_CreateSquare")]
        public void CreateSquare()
        {
            var document = Application.DocumentManager.MdiActiveDocument;
            var database = document.Database;
            var editor = document.Editor;

            // Prompt for the side length of the square
            PromptDoubleResult promptResult = editor.GetDouble("\nEnter side length of the square: ");
            if (promptResult.Status != PromptStatus.OK)
                return;

            double sideLength = promptResult.Value;

            // Prompt for the insertion point of the square
            PromptPointResult pointResult = editor.GetPoint("\nSpecify insertion point: ");
            if (pointResult.Status != PromptStatus.OK)
                return;

            // Get the point where the user clicked
            Point3d insertionPoint = pointResult.Value;

            // Create the square in the current space (Model or Paper space)
            using (Transaction transaction = database.TransactionManager.StartTransaction())
            {
                BlockTable blockTable = transaction.GetObject(database.BlockTableId, OpenMode.ForRead) as BlockTable;
                BlockTableRecord currentSpace = transaction.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                // Define the square as a polyline
                Polyline square = new Polyline();
                square.AddVertexAt(0, new Point2d(insertionPoint.X, insertionPoint.Y), 0, 0, 0);
                square.AddVertexAt(1, new Point2d(insertionPoint.X + sideLength, insertionPoint.Y), 0, 0, 0);
                square.AddVertexAt(2, new Point2d(insertionPoint.X + sideLength, insertionPoint.Y + sideLength), 0, 0, 0);
                square.AddVertexAt(3, new Point2d(insertionPoint.X, insertionPoint.Y + sideLength), 0, 0, 0);
                square.Closed = true;

                // Add the square to the current space
                currentSpace.AppendEntity(square);
                transaction.AddNewlyCreatedDBObject(square, true);

                // Commit the transaction
                transaction.Commit();
            }

            editor.WriteMessage($"\nSquare with side length {sideLength} created at ({insertionPoint.X}, {insertionPoint.Y}).");
        }


        [CommandMethod("PS_AlignObjects")]
        public void AlignObjects()
        {
            var document = Autodesk.AutoCAD.ApplicationServices.Application.DocumentManager.MdiActiveDocument;
            var editor = document.Editor;

            // Prompt user to select objects
            PromptSelectionResult selectionResult = editor.GetSelection();
            if (selectionResult.Status != PromptStatus.OK)
                return;

            // Get the selected objects
            SelectionSet selectionSet = selectionResult.Value;
            ObjectId[] objectIds = selectionSet.GetObjectIds();

            // Prompt user to select alignment type (horizontal or vertical)
            PromptKeywordOptions alignmentPromptOptions = new PromptKeywordOptions("\nSelect alignment direction [Horizontal/Vertical]:");
            alignmentPromptOptions.Keywords.Add("Horizontal");
            alignmentPromptOptions.Keywords.Add("Vertical");
            alignmentPromptOptions.Keywords.Default = "Horizontal";
            PromptResult alignmentPromptResult = editor.GetKeywords(alignmentPromptOptions);
            if (alignmentPromptResult.Status != PromptStatus.OK)
                return;

            string alignmentDirection = alignmentPromptResult.StringResult;
            bool isHorizontal = (alignmentDirection == "Horizontal");

            // Determine the offset distance based on user input
            PromptDoubleOptions offsetPromptOptions = new PromptDoubleOptions($"\nEnter offset distance (in {(isHorizontal ? "X" : "Y")} direction):");
            offsetPromptOptions.AllowNegative = true;
            offsetPromptOptions.AllowZero = true; // Allow zero spacing if needed
            PromptDoubleResult offsetPromptResult = editor.GetDouble(offsetPromptOptions);
            if (offsetPromptResult.Status != PromptStatus.OK)
                return;

            double offsetDistance = offsetPromptResult.Value;

            // Align the selected objects
            using (Transaction transaction = document.TransactionManager.StartTransaction())
            {
                BlockTableRecord currentSpace = transaction.GetObject(document.Database.CurrentSpaceId, OpenMode.ForWrite) as BlockTableRecord;

                // Get the points for sorting
                List<(ObjectId Id, Point3d Point)> objectPoints = new List<(ObjectId, Point3d)>();

                foreach (ObjectId objectId in objectIds)
                {
                    Entity entity = transaction.GetObject(objectId, OpenMode.ForRead) as Entity;
                    Point3d point = entity.GeometricExtents.MinPoint;
                    objectPoints.Add((objectId, point));
                }

                // Sort the objects based on the specified direction
                objectPoints.Sort((a, b) => isHorizontal ? a.Point.X.CompareTo(b.Point.X) : a.Point.Y.CompareTo(b.Point.Y));

                // Determine the reference point for alignment
                Point3d referencePoint = isHorizontal ? objectPoints[0].Point : objectPoints[0].Point;

                // Perform alignment and spacing
                double currentOffset = 0;

                foreach ((ObjectId objectId, Point3d startPoint) in objectPoints)
                {
                    Entity entity = transaction.GetObject(objectId, OpenMode.ForWrite) as Entity;
                    double offsetValue = isHorizontal ? referencePoint.Y - startPoint.Y : referencePoint.X - startPoint.X;

                    // Apply translation to align the object
                    Matrix3d translation = isHorizontal
                        ? Matrix3d.Displacement(new Vector3d(currentOffset - startPoint.X + referencePoint.X, offsetValue, 0))
                        : Matrix3d.Displacement(new Vector3d(offsetValue, currentOffset - startPoint.Y + referencePoint.Y, 0));

                    entity.TransformBy(translation);

                    // Update current offset for next object
                    currentOffset += offsetDistance + (isHorizontal ? entity.GeometricExtents.MaxPoint.X - entity.GeometricExtents.MinPoint.X : entity.GeometricExtents.MaxPoint.Y - entity.GeometricExtents.MinPoint.Y);
                }

                // Commit the transaction
                transaction.Commit();
            }

            editor.WriteMessage($"\nObjects aligned {alignmentDirection.ToLower()}ly with specified offset.");
        }

        // Helper method to get the bottom-most point of all selected objects
        private Point3d GetBottomMostPoint(Transaction transaction, ObjectId[] objectIds)
        {
            Point3d bottomMostPoint = new Point3d(double.MaxValue, double.MaxValue, 0.0); // Initialize with a large value

            foreach (ObjectId objectId in objectIds)
            {
                Entity entity = transaction.GetObject(objectId, OpenMode.ForRead) as Entity;
                Point3d bottomLeft = entity.GeometricExtents.MinPoint;
                if (bottomLeft.Y < bottomMostPoint.Y)
                {
                    bottomMostPoint = bottomLeft;
                }
            }

            return bottomMostPoint;
        }

        // Helper method to get the left-most point of all selected objects
        private Point3d GetLeftMostPoint(Transaction transaction, ObjectId[] objectIds)
        {
            Point3d leftMostPoint = new Point3d(double.MaxValue, double.MaxValue, 0.0); // Initialize with a large value

            foreach (ObjectId objectId in objectIds)
            {
                Entity entity = transaction.GetObject(objectId, OpenMode.ForRead) as Entity;
                Point3d topLeft = entity.GeometricExtents.MinPoint;
                if (topLeft.X < leftMostPoint.X)
                {
                    leftMostPoint = topLeft;
                }
            }

            return leftMostPoint;
        }





        [CommandMethod("CreateDeviceBox")]
        public void CreateDeviceBox()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Database db = doc.Database;
            Editor ed = doc.Editor;

            // Prompt for number of inputs and outputs
            PromptIntegerOptions inputsPromptOptions = new PromptIntegerOptions("\nEnter number of inputs:");
            inputsPromptOptions.AllowZero = false;
            PromptIntegerResult inputsPromptResult = ed.GetInteger(inputsPromptOptions);
            if (inputsPromptResult.Status != PromptStatus.OK)
                return;

            int numInputs = inputsPromptResult.Value;

            PromptIntegerOptions outputsPromptOptions = new PromptIntegerOptions("\nEnter number of outputs:");
            outputsPromptOptions.AllowZero = false;
            PromptIntegerResult outputsPromptResult = ed.GetInteger(outputsPromptOptions);
            if (outputsPromptResult.Status != PromptStatus.OK)
                return;

            int numOutputs = outputsPromptResult.Value;

            // Calculate box dimensions based on inputs and outputs
            double boxWidth = 8.0; // Default width
            double boxHeight = 2.0 + Math.Max(numInputs, numOutputs) * 1.5; // Adjusted height

            // Prompt for box label
            PromptStringOptions boxLabelPromptOptions = new PromptStringOptions("\nEnter box label:");
            PromptResult boxLabelPromptResult = ed.GetString(boxLabelPromptOptions);
            if (boxLabelPromptResult.Status != PromptStatus.OK)
                return;

            string boxLabel = boxLabelPromptResult.StringResult;

            // Start a transaction to create the box and labels
            using (Transaction tr = db.TransactionManager.StartTransaction())
            {
                BlockTable bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                BlockTableRecord btr = (BlockTableRecord)tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite);

                // Create the box
                Point3d corner = new Point3d(0, 0, 0); // Default corner point
                Polyline box = new Polyline();
                box.AddVertexAt(0, new Point2d(corner.X, corner.Y), 0, 0, 0);
                box.AddVertexAt(1, new Point2d(corner.X + boxWidth, corner.Y), 0, 0, 0);
                box.AddVertexAt(2, new Point2d(corner.X + boxWidth, corner.Y + boxHeight), 0, 0, 0);
                box.AddVertexAt(3, new Point2d(corner.X, corner.Y + boxHeight), 0, 0, 0);
                box.Closed = true;
                btr.AppendEntity(box);
                tr.AddNewlyCreatedDBObject(box, true);

                // Calculate text scale based on box dimensions
                double textScale = Math.Min(boxWidth, boxHeight) / 10.0;

                // Add inputs (lines going in from left)
                double inputSpacing = boxHeight / (numInputs + 1);
                for (int i = 0; i < numInputs; i++)
                {
                    Point3d startPoint = new Point3d(corner.X, corner.Y + (i + 1) * inputSpacing, corner.Z);
                    Point3d endPoint = new Point3d(corner.X - 2.0, corner.Y + (i + 1) * inputSpacing, corner.Z);
                    Line inputLine = new Line(startPoint, endPoint);
                    btr.AppendEntity(inputLine);
                    tr.AddNewlyCreatedDBObject(inputLine, true);

                    // Add label for input inside the box
                    MText inputLabel = new MText();
                    inputLabel.Contents = $"In {i + 1}";
                    inputLabel.Location = new Point3d(corner.X - 1.0, corner.Y + (i + 1) * inputSpacing, corner.Z);
                    inputLabel.TextHeight = textScale;
                    btr.AppendEntity(inputLabel);
                    tr.AddNewlyCreatedDBObject(inputLabel, true);
                }

                // Add outputs (lines going out from right)
                double outputSpacing = boxHeight / (numOutputs + 1);
                for (int i = 0; i < numOutputs; i++)
                {
                    Point3d startPoint = new Point3d(corner.X + boxWidth, corner.Y + (i + 1) * outputSpacing, corner.Z);
                    Point3d endPoint = new Point3d(corner.X + boxWidth + 2.0, corner.Y + (i + 1) * outputSpacing, corner.Z);
                    Line outputLine = new Line(startPoint, endPoint);
                    btr.AppendEntity(outputLine);
                    tr.AddNewlyCreatedDBObject(outputLine, true);

                    // Add label for output inside the box
                    MText outputLabel = new MText();
                    outputLabel.Contents = $"Out {i + 1}";
                    outputLabel.Location = new Point3d(corner.X + boxWidth + 1.0, corner.Y + (i + 1) * outputSpacing, corner.Z);
                    outputLabel.TextHeight = textScale;
                    btr.AppendEntity(outputLabel);
                    tr.AddNewlyCreatedDBObject(outputLabel, true);
                }

                // Add label for the box
                MText boxTextLabel = new MText();
                boxTextLabel.Contents = boxLabel;
                // Calculate leftmost point for the box label
                Point3d boxTextStart = new Point3d(corner.X - boxTextLabel.ActualWidth / 2.0, corner.Y + boxHeight + textScale * 2.0, corner.Z);
                boxTextLabel.Location = boxTextStart;
                boxTextLabel.TextHeight = textScale * 1.5; // Larger text for box label
                btr.AppendEntity(boxTextLabel);
                tr.AddNewlyCreatedDBObject(boxTextLabel, true);

                // Combine all entities into a block for easier selection
                DBObjectCollection entityObjs = new DBObjectCollection();
                entityObjs.Add(box);
                foreach (ObjectId id in btr)
                {
                    entityObjs.Add(id.GetObject(OpenMode.ForRead));
                }

                // Create a block table record for the block
                BlockTableRecord blockRecord = new BlockTableRecord();
                blockRecord.Name = "*U"; // Unique name
                bt.UpgradeOpen();
                ObjectId blockId = bt.Add(blockRecord);
                tr.AddNewlyCreatedDBObject(blockRecord, true);

                // Insert the block reference
                BlockReference blockRef = new BlockReference(boxTextStart, blockId);
                btr.AppendEntity(blockRef);
                tr.AddNewlyCreatedDBObject(blockRef, true);

                // Commit the transaction
                tr.Commit();
            }

            ed.WriteMessage($"\nDevice box '{boxLabel}' created with {numInputs} inputs and {numOutputs} outputs.");
        }

        [CommandMethod("PS_FanOutLines")]
        public void FanOutLines()
        {
            var document = Application.DocumentManager.MdiActiveDocument;
            var editor = document.Editor;

            // Prompt user to select the main polyline
            PromptEntityOptions polylinePromptOptions = new PromptEntityOptions("\nSelect a polyline:");
            polylinePromptOptions.SetRejectMessage("\nSelected entity is not a polyline.");
            polylinePromptOptions.AddAllowedClass(typeof(Polyline), true);
            PromptEntityResult polylinePromptResult = editor.GetEntity(polylinePromptOptions);
            if (polylinePromptResult.Status != PromptStatus.OK)
                return;

            ObjectId polylineId = polylinePromptResult.ObjectId;

            // Prompt user to select connection points
            PromptPointOptions pointPromptOptions = new PromptPointOptions("\nSelect points to connect (press Enter to finish):");
            pointPromptOptions.AllowNone = true;

            List<Point3d> connectionPoints = new List<Point3d>();
            while (true)
            {
                PromptPointResult pointPromptResult = editor.GetPoint(pointPromptOptions);
                if (pointPromptResult.Status == PromptStatus.None || pointPromptResult.Status == PromptStatus.Cancel)
                    break;

                if (pointPromptResult.Status == PromptStatus.OK)
                {
                    connectionPoints.Add(pointPromptResult.Value);
                }
            }

            if (connectionPoints.Count == 0)
            {
                editor.WriteMessage("\nNo connection points selected.");
                return;
            }

            using (Transaction transaction = document.TransactionManager.StartTransaction())
            {
                Polyline polyline = transaction.GetObject(polylineId, OpenMode.ForRead) as Polyline;
                BlockTable blockTable = transaction.GetObject(document.Database.BlockTableId, OpenMode.ForRead) as BlockTable;
                BlockTableRecord blockTableRecord = transaction.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForWrite) as BlockTableRecord;

                foreach (Point3d connectionPoint in connectionPoints)
                {
                    // Find the closest point on the polyline to the connection point
                    Point3d closestPoint = FindClosestPointOnPolyline(polyline, connectionPoint, transaction);

                    // Find the path to the closest point on the polyline
                    List<Line> pathLines = FindPathToClosestPoint(closestPoint, connectionPoint, transaction);

                    // Create new lines for the path
                    foreach (Line line in pathLines)
                    {
                        blockTableRecord.AppendEntity(line);
                        transaction.AddNewlyCreatedDBObject(line, true);
                    }
                }

                // Commit the transaction
                transaction.Commit();
            }

            editor.WriteMessage("\nLines created connecting to the selected points.");
        }

        // Helper method to find the closest point on a polyline to a given point
        private Point3d FindClosestPointOnPolyline(Polyline polyline, Point3d point, Transaction transaction)
        {
            Point3d closestPoint = new Point3d();
            double minDistance = double.MaxValue;

            for (int i = 0; i < polyline.NumberOfVertices - 1; i++)
            {
                Point3d start = polyline.GetPoint3dAt(i);
                Point3d end = polyline.GetPoint3dAt(i + 1);

                LineSegment3d segment = new LineSegment3d(start, end);
                PointOnCurve3d closestPointOnSegment = segment.GetClosestPointTo(point);

                Point3d closestPoint3dOnSegment = closestPointOnSegment.Point;
                double distance = closestPoint3dOnSegment.DistanceTo(point);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestPoint = closestPoint3dOnSegment;
                }
            }

            return closestPoint;
        }

        // Helper method to retrieve obstacles in the drawing
        private List<Entity> GetObstacles(Transaction transaction)
        {
            List<Entity> obstacles = new List<Entity>();
            BlockTable blockTable = transaction.GetObject(Application.DocumentManager.MdiActiveDocument.Database.BlockTableId, OpenMode.ForRead) as BlockTable;
            BlockTableRecord blockTableRecord = transaction.GetObject(blockTable[BlockTableRecord.ModelSpace], OpenMode.ForRead) as BlockTableRecord;

            foreach (ObjectId id in blockTableRecord)
            {
                Entity entity = transaction.GetObject(id, OpenMode.ForRead) as Entity;
                // Exclude lines, we will handle them separately
                if (entity != null && !(entity is Line))
                {
                    obstacles.Add(entity);
                }
            }

            return obstacles;
        }

        // Helper method to find the next point in the path to avoid obstacles
        private Point3d FindNextPoint(Point3d currentPoint, Point3d targetPoint, List<Entity> obstacles)
        {
            // Implement a simple grid-based approach or use A* algorithm to avoid obstacles and move horizontally or vertically
            // For simplicity, let's assume a basic implementation

            // Example implementation that moves horizontally or vertically
            //if (Math.Abs(targetPoint.X - currentPoint.X) > Math.Abs(targetPoint.Y - currentPoint.Y))
            if (targetPoint.X != currentPoint.X && targetPoint.Y != currentPoint.Y)
            {
                return new Point3d(currentPoint.X, targetPoint.Y, currentPoint.Z);
            }
            else
            {
                return new Point3d(targetPoint.X, currentPoint.Y, currentPoint.Z);
            }
        }

        // Helper method to find the path to the closest point on the polyline
        private List<Line> FindPathToClosestPoint(Point3d startPoint, Point3d endPoint, Transaction transaction)
        {
            //var document2 = Application.DocumentManager.MdiActiveDocument;
            //var editor2 = document2.Editor;
            List<Line> pathLines = new List<Line>();

            // Create a new pathfinding grid
            // Assume `GetObstacles` method retrieves all objects in the drawing that should be considered obstacles
            List<Entity> obstacles = GetObstacles(transaction);
            /*foreach (Entity entity in obstacles)
            {
                editor2.WriteMessage(entity.ToString() + "/n");
            }*/

            Point3d currentPoint = startPoint;
            Point3d targetPoint = endPoint;

            // Pathfinding logic
            while (currentPoint != targetPoint)
            {
                // Find the next point to move to (either horizontally or vertically)
                Point3d nextPoint = FindNextPoint(currentPoint, targetPoint, obstacles);

                // Create a line from current point to next point
                Line pathLine = new Line(currentPoint, nextPoint);
                pathLines.Add(pathLine);

                // Update the current point
                currentPoint = nextPoint;
            }

            // Add the final line to the end point
            Line finalLine = new Line(currentPoint, targetPoint);
            pathLines.Add(finalLine);

            return pathLines;
        }









        [CommandMethod("ConnectPoints")]
        public void ConnectPoints()
        {
            Document doc = Application.DocumentManager.MdiActiveDocument;
            Editor ed = doc.Editor;
            Database db = doc.Database;

            try
            {
                // Prompt user for the start point
                PromptPointResult ppr1 = ed.GetPoint("Select the start point: ");
                if (ppr1.Status != PromptStatus.OK) return;
                Point3d startPoint = ppr1.Value;

                // Prompt user for the end point
                PromptPointResult ppr2 = ed.GetPoint("Select the end point: ");
                if (ppr2.Status != PromptStatus.OK) return;
                Point3d endPoint = ppr2.Value;

                // Get bounding boxes of all objects in the drawing
                List<Extents3d> obstacles = new List<Extents3d>();
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    BlockTable bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                    BlockTableRecord btr = (BlockTableRecord)tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForRead);

                    foreach (ObjectId objId in btr)
                    {
                        Entity entity = (Entity)tr.GetObject(objId, OpenMode.ForRead);
                        if (entity.GeometricExtents != null)
                        {
                            obstacles.Add(entity.GeometricExtents);
                        }
                    }
                    tr.Commit();
                }

                // Implement pathfinding algorithm
                List<Point3d> path = FindPath(startPoint, endPoint, obstacles);

                if (path == null)
                {
                    ed.WriteMessage("No path found.");
                    return;
                }

                // Draw the path
                using (Transaction tr = db.TransactionManager.StartTransaction())
                {
                    BlockTable bt = (BlockTable)tr.GetObject(db.BlockTableId, OpenMode.ForRead);
                    BlockTableRecord btr = (BlockTableRecord)tr.GetObject(bt[BlockTableRecord.ModelSpace], OpenMode.ForWrite);

                    Polyline polyline = new Polyline();
                    for (int i = 0; i < path.Count; i++)
                    {
                        polyline.AddVertexAt(i, new Point2d(path[i].X, path[i].Y), 0, 0, 0);
                    }

                    btr.AppendEntity(polyline);
                    tr.AddNewlyCreatedDBObject(polyline, true);
                    tr.Commit();
                }
            }
            catch (System.Exception ex)
            {
                ed.WriteMessage("Error: " + ex.Message);
            }
        }

        private List<Point3d> FindPath(Point3d start, Point3d end, List<Extents3d> obstacles)
        {
            // Implement your pathfinding algorithm here
            List<Point3d> path = new List<Point3d>();

            // Using a simple A* algorithm with constraints
            PriorityQueue<Node> openSet = new PriorityQueue<Node>();
            Dictionary<Point3d, Node> allNodes = new Dictionary<Point3d, Node>();

            Node startNode = new Node(start, null, 0, GetHeuristic(start, end));
            openSet.Enqueue(startNode);
            allNodes[start] = startNode;

            while (openSet.Count > 0)
            {
                Node current = openSet.Dequeue();

                if (current.Point.IsEqualTo(end))
                {
                    Node temp = current;
                    while (temp != null)
                    {
                        path.Add(temp.Point);
                        temp = temp.Parent;
                    }
                    path.Reverse();
                    return path;
                }

                foreach (var neighbor in GetNeighbors(current.Point, obstacles))
                {
                    double tentativeGScore = current.GScore + GetDistance(current.Point, neighbor);
                    if (!allNodes.ContainsKey(neighbor) || tentativeGScore < allNodes[neighbor].GScore)
                    {
                        Node neighborNode = new Node(neighbor, current, tentativeGScore, GetHeuristic(neighbor, end));
                        openSet.Enqueue(neighborNode);
                        allNodes[neighbor] = neighborNode;
                    }
                }
            }

            // If we get here, no path was found
            return null;
        }

        private List<Point3d> GetNeighbors(Point3d point, List<Extents3d> obstacles)
        {
            List<Point3d> neighbors = new List<Point3d>
            {
                new Point3d(point.X + 1, point.Y, 0),
                new Point3d(point.X - 1, point.Y, 0),
                new Point3d(point.X, point.Y + 1, 0),
                new Point3d(point.X, point.Y - 1, 0)
            };

            // Remove neighbors that are within an obstacle
            neighbors.RemoveAll(n => IsPointInObstacle(n, obstacles));

            return neighbors;
        }

        private bool IsPointInObstacle(Point3d point, List<Extents3d> obstacles)
        {
            foreach (var obstacle in obstacles)
            {
                if (point.X >= obstacle.MinPoint.X && point.X <= obstacle.MaxPoint.X &&
                    point.Y >= obstacle.MinPoint.Y && point.Y <= obstacle.MaxPoint.Y)
                {
                    return true;
                }
            }
            return false;
        }

        private double GetDistance(Point3d a, Point3d b)
        {
            return Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y); // Manhattan distance for grid-based pathfinding
        }

        private double GetHeuristic(Point3d a, Point3d b)
        {
            return GetDistance(a, b); // Same as distance for now
        }

        private class Node : IComparable<Node>
        {
            public Point3d Point { get; }
            public Node Parent { get; }
            public double GScore { get; }
            public double FScore { get; }

            public Node(Point3d point, Node parent, double gScore, double fScore)
            {
                Point = point;
                Parent = parent;
                GScore = gScore;
                FScore = fScore;
            }

            public int CompareTo(Node other)
            {
                return FScore.CompareTo(other.FScore);
            }
        }

        private class PriorityQueue<T> where T : IComparable<T>
        {
            private List<T> data;

            public PriorityQueue()
            {
                this.data = new List<T>();
            }

            public void Enqueue(T item)
            {
                data.Add(item);
                int ci = data.Count - 1; // child index; start at end
                while (ci > 0)
                {
                    int pi = (ci - 1) / 2; // parent index
                    if (data[ci].CompareTo(data[pi]) >= 0) break; // child item is larger than (or equal) parent so we're done
                    T tmp = data[ci]; data[ci] = data[pi]; data[pi] = tmp;
                    ci = pi;
                }
            }

            public T Dequeue()
            {
                // Assumes pq is not empty
                int li = data.Count - 1; // last index (before removal)
                T frontItem = data[0];   // fetch the front
                data[0] = data[li];
                data.RemoveAt(li);

                --li; // last index (after removal)
                int pi = 0; // parent index. start at front of pq
                while (true)
                {
                    int ci = pi * 2 + 1; // left child index of parent
                    if (ci > li) break;  // no children so done
                    int rc = ci + 1;     // right child
                    if (rc <= li && data[rc].CompareTo(data[ci]) < 0) // if there is a right child (rc <= li) and it is smaller
                        ci = rc; // use the right child instead

                    if (data[pi].CompareTo(data[ci]) <= 0) break; // parent is smaller than (or equal to) smallest child so done
                    T tmp = data[pi]; data[pi] = data[ci]; data[ci] = tmp; // swap parent and child
                    pi = ci;
                }
                return frontItem;
            }

            public int Count
            {
                get { return data.Count; }
            }
        }









    }
}
