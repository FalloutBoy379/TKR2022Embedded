#include<stdio.h>
#include<stdlib.h>
#include<string.h>

#define MAX 30
#define TEMPORARY 1
#define PERMANENT 2
#define NIL -1
#define INFINITY 99999

struct Vertex
{
  char name[50];
  int status;
    int predecessor;
    int pathLength;
    float xcord;
    float ycord;
};

struct Vertex vertexList[MAX];

int adj[MAX][MAX]; 
int n=0;
int e=0; 


int getIndex(char s[]);
void insertVertex(char s[]);
void insertEdge(char s1[], char s2[], int wt);

void findPaths(char s[]);
void findPath(int s, int v);
void dijkstra(int s);
int tempVertexMinPL();

int counterpromax = 0;


void setup() 
{
    insertVertex("Zero");
    insertVertex("One");
    insertVertex("Two");
    insertVertex("Three");

    insertEdge("Zero", "One", 11);
    insertEdge("One", "Two", 12);
    insertEdge("Two", "Three", 13);

    insertEdge("One", "Zero", 11);
    insertEdge("Two", "One", 12);
    insertEdge("Three", "Two", 13);
/////////////////////////////////////////////////////////////////////////////////
//yahape co-ordinates daal jo sensor ke readings aaenge uske hisaab se
////////////////////////////////////////////////////////////////////////////////
/*
 AISA BANAYA HAI NODES MAINE
 0------------------------------------1
                                       \
                                        \
                                         \
                                          \
                                           2
                                           |
                                           |
                                           |
                                           |
                                           |
                                           |
                                           |
                                           |
                                           |
                                           |
                                           3
 */
    vertexList[0].xcord= 4050;
    vertexList[0].ycord= 0.0;
    vertexList[1].xcord= 1500;
    vertexList[1].ycord= 0.0;
    vertexList[2].xcord= 750;
    vertexList[2].ycord= 1500;
    vertexList[3].xcord= 750;
    vertexList[3].ycord= 3600;

    findPaths("Zero");//initial position to bal rack
    findPaths("Three");//ball rack to initial position
}

void loop() 
{
  
}


void findPaths(char source[])
{
     counterpromax+=1;
     int s,v;
   s = getIndex(source);

   dijkstra(s);

   if(counterpromax == 1)
     {
        findPath(s,3);
     }
     else if(counterpromax == 2)
     {
        findPath(s,0);
     }
}

void dijkstra(int s)
{
     int v, c;

     for(v = 0; v < n; v++)
     {
        vertexList[v].status = TEMPORARY;
        vertexList[v].pathLength = INFINITY;
        vertexList[v].predecessor = NIL;
     }

     vertexList[s].pathLength = 0;

     while(1)
     {
         c = tempVertexMinPL();

         if(c == NIL)
              return;

         vertexList[c].status = PERMANENT;

         for(v=0; v<n; v++)
         {
            if( adj[c][v]!=0 && vertexList[v].status == TEMPORARY )
                if( vertexList[c].pathLength + adj[c][v] < vertexList[v].pathLength)
                 {
                    vertexList[v].predecessor = c;
                    vertexList[v].pathLength = vertexList[c].pathLength + adj[c][v];
                 }
         }
      }
 }

 int tempVertexMinPL()
 {
     int min = INFINITY;
     int v, x = NIL;

   for (v = 0; v < n; v++)
     {
          if (vertexList[v].status == TEMPORARY && vertexList[v].pathLength < min)
          {
               min = vertexList[v].pathLength;
               x = v;
          }
      }
      return x;
}

void findPath(int s, int v)
{
  int i, u;
  int path[MAX];
    int sd = 0;
    int count = 0;

    while (v != s)
    {
       count++;
       path[count] = v;
       u = vertexList[v].predecessor;
       sd += adj[u][v];
       v=u;
    }
    count++;
    path[count] = s;
    for(i = count; i>=1; i--)
    {
          if(i!=1)
          {
          pid(vertexList[path[i-1]].xcord - vertexList[path[i]].xcord, vertexList[path[i-1]].ycord - vertexList[path[i]].ycord);
          }
    }
}



void pid(float x,float y)
{
    //x and y are targets, calculate errors accordingly and uspe PID karke call drive
}
int getIndex(char s[])
{
  int i;
    for (i = 0; i<n; i++)
       if ( strcmp(s,vertexList[i].name) == 0 )
         return i;
}

void insertVertex(char s[])
{
  strcpy(vertexList[n].name, s);
  n++;
}
void insertEdge(char s1[], char s2[], int wt)
{
    int u = getIndex(s1);
    int v = getIndex(s2);
         adj[u][v] = wt;
         e++;
}
