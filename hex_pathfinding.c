#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_AIR_ROUTES 5
#define COST_MIN 0
#define COST_MAX 100


#define HASH_SIZE 10007


typedef struct CacheNode {
  int xs, ys, xd, yd;
  int cost;
  struct CacheNode *next;
} CacheNode;


static CacheNode **cache = NULL;


static inline int hash_coords(int xs, int ys, int xd, int yd) {
  return ((xs * 31 + ys) * 31 + xd) * 31 + yd;
}

// liberare le liste della cache
static void cache_clear(void) {
  if (!cache)
    return;
  for (int i = 0; i < HASH_SIZE; ++i) {
    CacheNode *current = cache[i];
    while (current != NULL) {
      CacheNode *to_free = current;
      current = current->next;
      free(to_free);
    }
    cache[i] = NULL; // Il cassetto è ora vuoto
  }
}

// cercare nella cache scorrendo la lista corretta
static int cache_lookup(int xs, int ys, int xd, int yd) {
  if (!cache) // Se non esiste la cache, non trovo niente
    return -1;

  int h = hash_coords(xs, ys, xd, yd) % HASH_SIZE;
  CacheNode *current = cache[h]; //  Prendo il primo foglietto del cassetto

  while (current != NULL) {
    if (current->xs == xs && current->ys == ys && current->xd == xd &&
        current->yd == yd) {
      return current->cost;
    }
    current = current->next;
  }
  return -1;
}

// inserire nella cache aggiungendo un nodo in testa alla lista
static void cache_insert(int xs, int ys, int xd, int yd, int cost) {
  // Se la cache non esiste, la creo
  if (!cache) {
    cache = calloc(HASH_SIZE, sizeof(CacheNode *));
    if (!cache)
      return; // Se non riesco a crearla, esco
  }

  int h = hash_coords(xs, ys, xd, yd) % HASH_SIZE; // Trovo il cassetto giusto
  CacheNode *current = cache[h];                   // Prendo il primo foglietto

  // Controllo se esiste già un foglietto per questo viaggio
  while (current != NULL) {
    if (current->xs == xs && current->ys == ys && current->xd == xd &&
        current->yd == yd) {
      current->cost = cost; // Aggiorno il costo esistente
      return;
    }
    current = current->next;
  }

  // Creo un nuovo foglietto
  CacheNode *newNode = malloc(sizeof(CacheNode));
  if (!newNode)
    return;
  
  newNode->xs = xs;
  newNode->ys = ys;
  newNode->xd = xd;
  newNode->yd = yd;
  newNode->cost = cost;

  newNode->next = cache[h];
  cache[h] = newNode;
}

#pragma pack(push, 1)
typedef struct {
  uint16_t toX, toY;
  uint8_t cost;
} AirRoute;

typedef struct {
  uint8_t landCost;
  AirRoute airRoutes[MAX_AIR_ROUTES];
  uint8_t airCount;
} Hex;
#pragma pack(pop)

typedef struct {
  int idx, dist;
} NodePQ;

static Hex **grid = NULL; // Matrice di celle esagonali
static int nRows = 0, nCols = 0;

// Buffer globali riusabili per travel_cost
static int *dist = NULL;
static unsigned char *vis = NULL; // Array di flag per i nodi visitati
static NodePQ *heap = NULL;
static size_t capN = 0; // capacità dell'array

static void ensure_workspace(size_t N) {
  if (N > capN) {
    dist = realloc(dist, N * sizeof *dist);
    vis = realloc(vis, N * sizeof *vis);
    heap = realloc(heap, N * sizeof *heap);
    capN = N;
  }
}

static void free_grid(void) {
  
  if (grid) {
    for (int r = 0; r < nRows; ++r)
      free(grid[r]);
    free(grid);
  }
  grid = NULL;
  nRows = nCols = 0;
  cache_clear(); // pulisce cache quando griglia cambia
  // libero buffer globali
  free(dist);
  dist = NULL;
  free(vis);
  vis = NULL;
  free(heap);
  heap = NULL;
  capN = 0;
}

static int alloc_grid(int rows, int cols) {
  Hex **tmp = malloc(rows * sizeof *tmp); // creo un array di puntatori a righe
  if (!tmp)
    return -1;

  for (int r = 0; r < rows; ++r) {
    tmp[r] = calloc(cols, sizeof *tmp[r]); // per ogni riga creo un array di
                                           // cols inizializzato a zero
    if (!tmp[r]) {
      for (int i = 0; i < r; ++i)
        free(tmp[i]);
      free(tmp);
      return -1;
    }
  }
  grid = tmp;
  nRows = rows;
  nCols = cols;
  return 0;
}

// convertire le coordinate offset nelle coordinate cubiche
static inline void offset_to_cube(int x, int y, int *cx, int *cy, int *cz) {
  *cx = x - (y - (y & 1)) / 2; //& per sapere se y è pari o dispari
  *cz = y;
  *cy = -(*cx) - (*cz); // in coordinate cubiche vale la somma delle tre
                        // coordinate pari a zero
}

// calcolare la distanza tra due celle esagonali
static int hex_distance(int x1, int y1, int x2, int y2) {
  int x1c, y1c, z1c, x2c, y2c, z2c;
  offset_to_cube(x1, y1, &x1c, &y1c,
                 &z1c); // converto le coordinate offset in cubiche
  offset_to_cube(x2, y2, &x2c, &y2c, &z2c);
  int dx = abs(x1c - x2c);
  int dy = abs(y1c - y2c);
  int dz = abs(z1c - z2c);
  return (dx + dy + dz) / 2; // formula per la distanza esagonale
}

// coordinate dei 6 vicini, even per righe pari, odd per righe dispari
static const int neigh_even[6][2] = {{+1, 0}, {0, -1},  {-1, -1},
                                     {-1, 0}, {-1, +1}, {0, +1}};
static const int neigh_odd[6][2] = {{+1, 0}, {+1, -1}, {0, -1},
                                    {-1, 0}, {0, +1},  {+1, +1}};

// aggiornare il costo con delta e limitare al range [COST_MIN,COST_MAX]
static void apply_delta(uint8_t *costPtr, int delta) {
  int v = *costPtr + delta;
  if (v < COST_MIN)
    v = COST_MIN;
  if (v > COST_MAX)
    v = COST_MAX;
  *costPtr = (uint8_t)v;
}

static void init(int cols, int rows) {
  free_grid(); // libero la griglia esistente
  if (cols <= 0 || rows <= 0 || alloc_grid(rows, cols) != 0) {
    fprintf(stderr, "Errore: memoria insufficiente\n");
    exit(EXIT_FAILURE);
  }
  for (int y = 0; y < rows; ++y)
    for (int x = 0; x < cols; ++x) {
      grid[y][x].landCost = 1;
      grid[y][x].airCount = 0;
    }
  puts("OK");
}

static void change_cost(int cx, int cy, int v, int radius) {
  // controllo validità degli argomenti
  if (radius <= 0 || cx < 0 || cx >= nCols || cy < 0 || cy >= nRows ||
      v < -10 || v > 10) {
    puts("KO");
    return;
  }
  // modifico costi terreno e rotta aerea
  for (int y = cy - (radius - 1); y <= cy + (radius - 1); ++y) {
    if (y < 0 || y >= nRows)
      continue;
    for (int x = cx - (radius - 1); x <= cx + (radius - 1); ++x) {
      if (x < 0 || x >= nCols)
        continue;

      int d = hex_distance(cx, cy, x, y);
      if (d >= radius)
        continue;

      int delta = (int)floor(v * (double)(radius - d) / radius);
      apply_delta(&grid[y][x].landCost, delta);
      for (int i = 0; i < grid[y][x].airCount; ++i)
        apply_delta(&grid[y][x].airRoutes[i].cost,
                    delta); // aggiorno tutte le rotte aeree
    }
  }
  cache_clear(); // pulisce cache quando costi cambiano
  puts("OK");
}

static void toggle_air_route(int x1, int y1, int x2, int y2) {
 
  if (x1 < 0 || x1 >= nCols || y1 < 0 || y1 >= nRows || x2 < 0 || x2 >= nCols ||
      y2 < 0 || y2 >= nRows || (x1 == x2 && y1 == y2)) {
    puts("KO");
    return;
  }

  Hex *from = &grid[y1][x1]; // puntatore alla cella di partenza

  for (int i = 0; i < from->airCount; ++i) {
    // se trovo una rotta che va verso la destinazione la elimino
    if (from->airRoutes[i].toX == x2 && from->airRoutes[i].toY == y2) {
      from->airRoutes[i] = from->airRoutes[from->airCount - 1];
      from->airCount--;
      cache_clear(); // pulire cache quando rotte cambiano
      puts("OK");
      return;
    }
  }

  if (from->airCount >= MAX_AIR_ROUTES) {
    puts("KO");
    return;
  }

  // calcolo costo della nuova rotta
  long sum = from->landCost;
  for (int i = 0; i < from->airCount; ++i)
    sum += from->airRoutes[i].cost;

  int newCost = (int)(sum / (from->airCount + 1));
  if (newCost < COST_MIN)
    newCost = COST_MIN;
  else if (newCost > COST_MAX)
    newCost = COST_MAX;
  
  from->airRoutes[from->airCount++] = (AirRoute){x2, y2, newCost};
  cache_clear();
  puts("OK");
}

// inserire un nuovo nodo nella heap
static void pq_push(NodePQ *heap, int *hn, NodePQ v) {
  int i = (*hn)++; // indice del nuovo nodo
  heap[i] = v;     // metto il nuovo nodo in fondo
  while (i > 0) {
    int p = (i - 1) / 2; // heapify up
    if (heap[p].dist <= heap[i].dist)
      break;
    NodePQ t = heap[p];
    heap[p] = heap[i];
    heap[i] = t;
    i = p;
  }
}
// rimuovere e restituisce l'elemento con priorità minima
static NodePQ pq_pop(NodePQ *heap, int *hn) {
  NodePQ res = heap[0]; 
  heap[0] =
      heap[--(*hn)]; 
  int i = 0;
  for (;;) { // heapify down
    int l = 2 * i + 1, r = l + 1,
        m = i; // l=figlio sx, r=figlio dx, m=minimo tra i,l,r
    if (l < *hn && heap[l].dist < heap[m].dist)
      m = l;
    if (r < *hn && heap[r].dist < heap[m].dist)
      m = r;
    if (m == i)
      break;
    NodePQ t = heap[i];
    heap[i] = heap[m];
    heap[m] = t;
    i = m;
  }
  return res;
}

static void travel_cost(int xs, int ys, int xd, int yd) {
  // controllo validità degli argomenti
  if (xs < 0 || xs >= nCols || ys < 0 || ys >= nRows || xd < 0 || xd >= nCols ||
      yd < 0 || yd >= nRows) {
    puts("-1");
    return;
  }
  // se parto e arrivo nello stesso punto, il costo è 0
  if (xs == xd && ys == yd) {
    puts("0");
    return;
  }

  // controllo cache: cerca se abbiamo già calcolato questo percorso
  int cached_result = cache_lookup(xs, ys, xd, yd);
  if (cached_result != -1) {
    printf("%d\n", cached_result);
    return;
  }

  const int N = nRows * nCols;
  const int start = ys * nCols + xs; // indice lineare
  const int destIdx = yd * nCols + xd;

  ensure_workspace(N); 
  memset(vis, 0, N * sizeof(*vis)); 
  memset(dist, 0x3f, N * sizeof *dist); // 0x3f3f3f3f ≈ 1 061 109 567
  dist[start] = 0;

  int hn = 0;
  pq_push(heap, &hn, (NodePQ){start, 0}); // inserisco il nodo di partenza

  while (hn) {
    NodePQ cur = pq_pop(heap, &hn); // prendo il nodo con priorità minima
    int u = cur.idx;
    if (vis[u]) 
      continue;
    if (cur.dist !=
        dist[u]) 
      continue;

    vis[u] = 1;
    if (u == destIdx) {
      // Early termination
      break;
    }

    int uq = u % nCols, ur = u / nCols; // riga,colonna corrente
    Hex *hex = &grid[ur][uq];
    int du = dist[u]; // costo per arrivare a u

    if (grid[ur][uq].landCost == 0 && u != start && u != destIdx) {
      continue;
    }
    // espansione verso celle adiacenti via terra
    if (hex->landCost > 0) {
      const int (*NBR)[2] =
          (ur & 1) ? neigh_odd : neigh_even; // usa il giusto array di vicini
      for (int k = 0; k < 6; ++k) {
        int vq = uq + NBR[k][0]; // colonna del vicino
        int vr = ur + NBR[k][1]; // riga del vicino
        if (vq < 0 || vq >= nCols || vr < 0 ||
            vr >= nRows) // se il vicino è fuori dalla griglia, lo skippo
          continue;
        int v = vr * nCols + vq;
        if (vis[v])
          continue;

        if (grid[vr][vq].landCost == 0 && v != destIdx)
          continue;

        int nd = du + (int)hex->landCost; // costo per arrivare al vicino
        // se ho trovato una soluzione migliore aggiornala
        if (nd < dist[v]) {
          dist[v] = nd;
          pq_push(heap, &hn, (NodePQ){v, nd});
        }
      }
    }
    // espansione tramite rotte aeree
    for (int i = 0; i < hex->airCount; ++i) {
      AirRoute *ar = &hex->airRoutes[i];
      int v = ar->toY * nCols + ar->toX;
      if (vis[v])
        continue;

      if (ar->cost <= 0)
        continue;

      int nd = du + (int)ar->cost;
      if (nd < dist[v]) {
        dist[v] = nd;
        pq_push(heap, &hn, (NodePQ){v, nd});
      }
    }
  }

  int ans = dist[destIdx];

  // se il costo è finito, lo salvo nella cache
  if (ans != 0x3f3f3f3f) {
    cache_insert(xs, ys, xd, yd, ans);
  }

  if (ans == 0x3f3f3f3f)
    puts("-1");
  else
    printf("%d\n", ans);
}

int main(void) {
  char cmd[64];
  while (scanf("%63s", cmd) == 1) {
    if (strcmp(cmd, "init") == 0) {
      int c, r;
      if (scanf("%d %d", &c, &r) != 2) {
        while (getchar() != '\n' && !feof(stdin))
          ;
        continue;
      }
      init(c, r);
    } else if (strcmp(cmd, "change_cost") == 0) {
      int x, y, v, rad;
      if (scanf("%d %d %d %d", &x, &y, &v, &rad) != 4) {
        while (getchar() != '\n' && !feof(stdin))
          ;
        continue;
      }
      change_cost(x, y, v, rad);
    } else if (strcmp(cmd, "toggle_air_route") == 0) {
      int x1, y1, x2, y2;
      if (scanf("%d %d %d %d", &x1, &y1, &x2, &y2) != 4) {
        while (getchar() != '\n' && !feof(stdin))
          ;
        continue;
      }
      toggle_air_route(x1, y1, x2, y2);
    } else if (strcmp(cmd, "travel_cost") == 0) {
      int xs, ys, xd, yd;
      if (scanf("%d %d %d %d", &xs, &ys, &xd, &yd) != 4) {
        while (getchar() != '\n' && !feof(stdin))
          ;
        continue;
      }
      travel_cost(xs, ys, xd, yd);
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
      break;
    } else {
      while (getchar() != '\n' && !feof(stdin))
        ;
    }
  }
  free_grid();
  free(cache);
  return 0;
}