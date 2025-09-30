using System.ComponentModel;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace SearchAlgorithms
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {

        

        public MainWindow()
        {
            InitializeComponent();
            CreateGrid();

            btnRunAStar.Click += BtnRunAStar_Click;
            btnClear.Click += BtnClear_Click;
            canvas.MouseLeftButtonDown += Canvas_MouseLeftButtonDown;
            canvas.MouseLeftButtonUp += (s, e) => isDrawing = false;
            canvas.MouseMove += Canvas_MouseMove;
        }


        /* Node */

        Node? start = null;
        Node? goal = null;


        public class Node
        {
            public int posX, posY;
            public double scoreG, scoreH;
            public double scoreF => scoreG + scoreH; //being used for A* for now
            public bool visited, obstacle;
            public Node? parent = null; //used for retrieving path
            public List<Node> neighbors = new List<Node>();

        }

        /* Grid */
        const int GridSize = 32;
        const int CellSize = 20;
        public Node[,] grid = new Node[GridSize, GridSize];
        Rectangle[,] rects = new Rectangle[GridSize, GridSize];
        bool isDrawing = false;

        private void CreateGrid()
        {
            canvas.Children.Clear();
            for (int x = 0; x < GridSize; x++)
            {
                for (int y = 0; y < GridSize; y++)
                {
                    //Initialize every node clear
                    grid[x, y] = new Node { posX = x, posY = y, neighbors = [], visited = false, obstacle = false, parent = null, scoreG = 0, scoreH = 0};

                    Rectangle r = new Rectangle
                    {
                        Width = CellSize - 1,
                        Height = CellSize - 1,
                        Stroke = Brushes.LightGray,
                        Fill = Brushes.White
                    };

                    Canvas.SetLeft(r, x * CellSize);
                    Canvas.SetTop(r, y * CellSize);
                    rects[x, y] = r;
                    canvas.Children.Add(r);

                }
            }

        }
        private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            Point p = e.GetPosition(canvas);
            int x = (int)(p.X / CellSize);
            int y = (int)(p.Y / CellSize);
            if (x < 0 || y < 0 || x >= GridSize || y >= GridSize) return;

            Node n = grid[x, y];

            if (start == null)
            {
                start = n;
                rects[x, y].Fill = Brushes.Green;
            }
            else if (goal == null && n != start)
            {
                goal = n;
                rects[x, y].Fill = Brushes.Red;
            }
            else if (n != start && n != goal)
            {
                ToggleObstacle(n);
            }

            isDrawing = true;
        }

        private void Canvas_MouseMove(object sender, MouseEventArgs e)
        {
            if (!isDrawing) return;
            Point p = e.GetPosition(canvas);
            int x = (int)(p.X / CellSize);
            int y = (int)(p.Y / CellSize);
            if (x < 0 || y < 0 || x >= GridSize || y >= GridSize) return;

            Node n = grid[x, y];
            if (n != start && n != goal)
            {
                n.obstacle = true;
                rects[x, y].Fill = Brushes.Black;
            }
        }

        private void ToggleObstacle(Node n)
        {
            n.obstacle = !n.obstacle;
            rects[n.posX, n.posY].Fill = n.obstacle ? Brushes.Black : Brushes.White;
        }

        private void BtnRunAStar_Click(object sender, RoutedEventArgs e)
        {
            if (start == null || goal == null) return;

            var astar = new AStar(grid);

            
            AStar.HeuristicFunc heuristic = ManhattanHeuristic;
            bool allowDiagonal = false;

            string selected = ((ComboBoxItem)heuristicSelector.SelectedItem).Content.ToString();
            if (selected == "Euclidean")
            {
                heuristic = EuclideanDistance;
                allowDiagonal = true;  
            }

            var (path, expanded) = astar.FindPath(start, goal, heuristic, allowDiagonal);

            
            foreach (var n in expanded)
            {
                if (n != start && n != goal && !path.Contains(n))
                    rects[n.posX, n.posY].Fill = Brushes.LightBlue;
            }

            
            foreach (var n in path)
            {
                if (n != start && n != goal)
                    rects[n.posX, n.posY].Fill = Brushes.DarkBlue;
            }
        }

        private void BtnClear_Click(object sender, RoutedEventArgs e)
        {
            start = null;
            goal = null;
            CreateGrid();
        }

        /* Heuristics */
        public static double ManhattanHeuristic(Node node1, Node node2)
        {
            return (Math.Abs(node1.posX - node2.posX) + Math.Abs(node1.posY - node2.posY));
        }

        public static double EuclideanDistance(Node node1, Node node2)
        {
            return Math.Sqrt(
                Math.Pow((node1.posX - node2.posX), 2) +
                Math.Pow((node1.posY - node2.posY), 2));
        }

        /* Algorithms */

        /* A* */
        public class AStar
        {
            public List<Node> openSet = new List<Node>();
            public List<Node> closedSet = new List<Node>();
            public List<Node> expanded = new List<Node>();
            private Node[,] grid;
            public delegate double HeuristicFunc(Node a, Node b);

            public AStar(Node[,] grid)
            {
                this.grid = grid;
            }

            public (List<Node> path, List<Node> expanded) FindPath(Node start, Node target, HeuristicFunc heuristic, bool allowDiagonal)
            {
                openSet.Add(start);
                start.parent = null;
                start.scoreG = 0;
                start.scoreH = heuristic(start, target);

                while(openSet.Count > 0)
                {
                    Node current = openSet.OrderBy(n => n.scoreF).First();

                    
                    if(current == target)
                    {
                        List<Node> path = new List<Node>();
                        current = target;
                        while(current!=null) // first node parent is null
                        {
                            path.Add(current);
                            current = current.parent; // retrocedes the path 

                        }
                        path.Reverse(); //reverse it to start-end 
                        return (path, expanded);
                    }

                    openSet.Remove(current);
                    closedSet.Add(current);
                    List<Node> neighbors = GetNeighbors(current, grid, allowDiagonal);
                    
                    foreach(var neighbor in neighbors)
                    {
                        if (neighbor.obstacle || closedSet.Contains(neighbor)) continue;
                        double moveCost = (neighbor.posX != current.posX && neighbor.posY != current.posY) ? Math.Sqrt(2) : 1.0;
                        if (!openSet.Contains(neighbor))
                        {
                            neighbor.scoreG = current.scoreG + moveCost;
                            neighbor.scoreH = heuristic(neighbor, target);
                            neighbor.visited = true;
                            neighbor.parent = current;
                            openSet.Add(neighbor);
                        }

                    }
                    
                }
                return (new List<Node>(), expanded);
            }

            public List<Node> GetNeighbors(Node current, Node[,] grid, bool allowDiagonal = false)
            {
                List<Node> neighbors = new List<Node>();
                int gridSize = grid.GetLength(0);

                int x = current.posX;
                int y = current.posY;

                if (x > 0) neighbors.Add(grid[x - 1, y]);
                if (x < gridSize - 1) neighbors.Add(grid[x + 1, y]);
                if (y > 0) neighbors.Add(grid[x, y - 1]);
                if (y < gridSize - 1) neighbors.Add(grid[x, y + 1]);

                if (allowDiagonal)
                {
                    if (x > 0 && y > 0) neighbors.Add(grid[x - 1, y - 1]);
                    if (x < gridSize - 1 && y > 0) neighbors.Add(grid[x + 1, y - 1]);
                    if (x > 0 && y < gridSize - 1) neighbors.Add(grid[x - 1, y + 1]);
                    if (x < gridSize - 1 && y < gridSize - 1) neighbors.Add(grid[x + 1, y + 1]);
                }
                return neighbors;
            }

        }

    }
}