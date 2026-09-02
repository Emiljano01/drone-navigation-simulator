import pygame
import math
import heapq
import random
import sys
import time


COLS, ROWS   = 28, 22
TILE_W       = 64          
TILE_H       = 32          
BLOCK_H      = 36          
DRONE_H      = 48          
WIN_W        = 1280
WIN_H        = 720
FPS          = 60
SENSOR_RANGE = 6
DRONE_SPEED  = 2         


ISO_OX = WIN_W // 2
ISO_OY = 120


SKY_TOP   = (8,  12,  30)
SKY_BOT   = (15, 22,  55)
FLOOR_TOP = (18, 32,  65)
FLOOR_L   = (12, 24,  50)
FLOOR_R   = (10, 20,  42)
GRID_COL  = (25, 45,  90)

BLK_TOP   = (55, 80, 160)
BLK_L     = (30, 50, 110)
BLK_R     = (20, 38,  88)
BLK_EDGE  = (80,120, 220)

TARGET_COL  = (0,  255, 140)
DRONE_TOP   = (0,  200, 255)
DRONE_SIDE  = (0,  130, 200)
ROTOR_COL   = (0,  230, 255)
ROTOR_ON    = (255,230,  80)
ARM_COL     = (0,  160, 210)
LED_COL     = (0,  255, 180)

HUD_BG      = (8,  14,  32)
HUD_BORDER  = (0,  180, 255)
ACCENT      = (0,  220, 255)
GREEN       = (30, 255, 120)
AMBER       = (255,180,  30)
RED_COL     = (255, 60,  60)
DIM         = (60,  85, 130)
WHITE       = (200,220, 245)
TRAIL_COL   = (0,  160, 255)
PATH_COL    = (30, 255, 120)
SHADOW_COL  = (0,   0,   0)

SENSOR_DIRS = [
    ( 0,-1,"N"),( 0, 1,"S"),( 1, 0,"E"),(-1, 0,"W"),
    ( 1,-1,"NE"),(-1,-1,"NW"),( 1, 1,"SE"),(-1, 1,"SW"),
]


def iso(gx, gy, gz=0):
    """Grid coord → pixel (izometrik)."""
    sx = ISO_OX + (gx - gy) * (TILE_W // 2)
    sy = ISO_OY + (gx + gy) * (TILE_H // 2) - gz
    return sx, sy

def iso_inv(px, py):
    """Pixel → grid coord (inversi)."""
    px -= ISO_OX
    py -= ISO_OY
    gx = (px / (TILE_W/2) + py / (TILE_H/2)) / 2
    gy = (py / (TILE_H/2) - px / (TILE_W/2)) / 2
    return int(round(gx)), int(round(gy))


def heuristic(a, b):
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return max(dx,dy) + (math.sqrt(2)-1)*min(dx,dy)

def astar(start, goal, obstacles):
    DIRS = [(0,-1,1),(0,1,1),(1,0,1),(-1,0,1),
            (1,-1,1.41),(-1,-1,1.41),(1,1,1.41),(-1,1,1.41)]
    heap = [(0, start)]
    came = {}
    g    = {start: 0}
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            path = []
            while cur in came:
                path.append(cur); cur = came[cur]
            path.append(start); path.reverse()
            return path
        for dx,dy,c in DIRS:
            nb = (cur[0]+dx, cur[1]+dy)
            if not(0<=nb[0]<COLS and 0<=nb[1]<ROWS): continue
            if nb in obstacles: continue
            ng = g[cur]+c
            if nb not in g or ng < g[nb]:
                came[nb]=cur; g[nb]=ng
                heapq.heappush(heap,(ng+heuristic(nb,goal),nb))
    return None


def draw_tile(surf, gx, gy, top_col, l_col, r_col, h=0, edge_col=None):
    """Vizato një tile izometrik me opsion lartësie."""
    cx, cy = iso(gx, gy, h)
    tw, th = TILE_W//2, TILE_H//2
    
    top = [
        (cx,      cy - th),
        (cx + tw, cy),
        (cx,      cy + th),
        (cx - tw, cy),
    ]
    pygame.draw.polygon(surf, top_col, top)

    if h > 0:
        
        left = [
            (cx - tw, cy),
            (cx,      cy + th),
            (cx,      cy + th + h),
            (cx - tw, cy + h),
        ]
        pygame.draw.polygon(surf, l_col, left)
        
        right = [
            (cx,      cy + th),
            (cx + tw, cy),
            (cx + tw, cy + h),
            (cx,      cy + th + h),
        ]
        pygame.draw.polygon(surf, r_col, right)

    if edge_col:
        pygame.draw.polygon(surf, edge_col, top, 1)

def draw_block(surf, gx, gy, height=BLOCK_H, variation=0):
    """Vizato bllok 3D me lartësi."""
    
    v = variation * 8
    top = (min(255,BLK_TOP[0]+v), min(255,BLK_TOP[1]+v), min(255,BLK_TOP[2]+v))
    draw_tile(surf, gx, gy, top, BLK_L, BLK_R, height, BLK_EDGE)
    
    cx, cy = iso(gx, gy, height//2)
    if height > 20:
        for wrow in range(min(2, height//18)):
            for wcol in [-8, 8]:
                wx = cx + wcol
                wy = cy - wrow*14 + 6
                wc = (100,180,255) if random.random() > 0.3 else (30,50,90)
                pygame.draw.rect(surf, wc, (wx-4, wy-4, 8, 8))
                pygame.draw.rect(surf, BLK_EDGE, (wx-4, wy-4, 8, 8), 1)


class Drone:
    def __init__(self, x, y):
        self.gx, self.gy = x, y
        self.wx = float(x)  
        self.wy = float(y)  
        self.wz = float(DRONE_H)

        self.path     = []
        self.pidx     = 0
        self.flying   = False
        self.reached  = False
        self.battery  = 100.0
        self.steps    = 0
        self.velocity = 0.0

        self.rotor_a  = 0.0
        self.bob      = 0.0
        self.bob_t    = 0.0
        self.heat     = 0.0
        self.t        = 0.0

        self.trail    = []   
        self.sensors  = {d[2]: SENSOR_RANGE for d in SENSOR_DIRS}

        self._obs_ref = set()

    def set_path(self, path):
        self.path = path; self.pidx = 0
        self.flying = True; self.reached = False; self.steps = 0

    def update_sensors(self, obstacles):
        gx, gy = int(round(self.wx)), int(round(self.wy))
        for dx, dy, name in SENSOR_DIRS:
            dist = SENSOR_RANGE
            for s in range(1, SENSOR_RANGE+1):
                nx, ny = gx+dx*s, gy+dy*s
                if not(0<=nx<COLS and 0<=ny<ROWS) or (nx,ny) in obstacles:
                    dist = s-1; break
            self.sensors[name] = dist

    def update(self, obstacles, dt):
        self.t     += dt
        self.bob_t += dt
        self.bob    = math.sin(self.bob_t * 2.8) * 3.0
        spin = 9.0 if self.flying else 1.2
        self.rotor_a = (self.rotor_a + spin * dt * 60) % 360

        if self.flying:
            self.heat = min(1.0, self.heat + dt * 0.8)
        else:
            self.heat = max(0.0, self.heat - dt * 0.5)

        self._obs_ref = obstacles
        self.update_sensors(obstacles)

        if not self.flying or self.pidx >= len(self.path):
            self.velocity = 0.0; return

        nc   = self.path[self.pidx]
        tx   = float(nc[0]); ty = float(nc[1])
        ddx  = tx - self.wx; ddy = ty - self.wy
        dist = math.hypot(ddx, ddy)
        spd  = DRONE_SPEED * dt * 60 / (TILE_W/2)  

        if dist < spd + 0.01:
            self.wx = tx; self.wy = ty
            self.gx, self.gy = nc
            self.trail.append((self.wx, self.wy, DRONE_H))
            if len(self.trail) > 150: self.trail.pop(0)
            self.pidx += 1; self.steps += 1
            self.battery = max(0.0, self.battery - 0.4)
            self.velocity = spd * (TILE_W/2) / dt / 60 * 3.6
            if self.pidx >= len(self.path):
                self.flying = False; self.reached = True
        else:
            self.wx += ddx/dist * spd
            self.wy += ddy/dist * spd
            self.velocity = DRONE_SPEED * 3.6 * 0.5

    def draw(self, surf, font_xs):
       
        if len(self.trail) > 1:
            pts = []
            for i,(tx,ty,tz) in enumerate(self.trail):
                px, py = iso(tx, ty, tz)
                pts.append((px, py))
            for i in range(1, len(pts)):
                t = i / len(self.trail)
                a = max(0, int(t * 180))
                c = (int(TRAIL_COL[0]*t), int(TRAIL_COL[1]*t), int(TRAIL_COL[2]*t))
                pygame.draw.line(surf, c, pts[i-1], pts[i], 2)

        
        if self.path and self.pidx < len(self.path):
            wpts = []
            for p in self.path[self.pidx:]:
                px, py = iso(p[0], p[1], DRONE_H)
                wpts.append((px, py))
            if len(wpts) > 1:
                pygame.draw.lines(surf, (20, 180, 80), False, wpts, 1)
                for wp in wpts[::4]:
                    pygame.draw.circle(surf, PATH_COL, wp, 3)

        sx, sy = iso(self.wx, self.wy, 0)
        hw = TILE_W//3; hh = TILE_H//4
        shadow_surf = pygame.Surface((hw*2, hh*2), pygame.SRCALPHA)
        pygame.draw.ellipse(shadow_surf, (0,0,0,70), (0,0,hw*2,hh*2))
        surf.blit(shadow_surf, (sx-hw, sy-hh))

  
        px, py = iso(self.wx, self.wy, DRONE_H + self.bob)
        pygame.draw.line(surf, (0,80,160), (sx,sy), (px, py), 1)

        cx, cy = px, py

        arm_dirs = [
            ( 0.6, -0.3), (-0.6, -0.3),
            ( 0.6,  0.3), (-0.6,  0.3),
        ]
        arm_pix = [(int(cx + a[0]*TILE_W//2*0.45),
                    int(cy + a[1]*TILE_H//2*0.9 + a[0]*0.1*TILE_H//2))
                   for a in arm_dirs]

        for ap in arm_pix:
            pygame.draw.line(surf, DRONE_SIDE, (cx,cy), ap, 3)

        rotor_col = ROTOR_ON if self.flying else ROTOR_COL
        for ap in arm_pix:
            for off in [0, 90]:
                ra = math.radians(self.rotor_a + off)
                r1x = int(ap[0] + 9*math.cos(ra)*0.9)
                r1y = int(ap[1] + 9*math.sin(ra)*0.45)
                r2x = int(ap[0] - 9*math.cos(ra)*0.9)
                r2y = int(ap[1] - 9*math.sin(ra)*0.45)
                pygame.draw.line(surf, rotor_col, (ap[0],ap[1]),(r1x,r1y), 3)
                pygame.draw.line(surf, rotor_col, (ap[0],ap[1]),(r2x,r2y), 3)
            pygame.draw.circle(surf, ACCENT, ap, 3)

        body_w, body_h = 18, 10
        pygame.draw.ellipse(surf, DRONE_SIDE,
                            (cx-body_w, cy-body_h, body_w*2, body_h*2))
        pygame.draw.ellipse(surf, DRONE_TOP,
                            (cx-body_w+2, cy-body_h+2, body_w*2-4, body_h*2-4))
        pygame.draw.ellipse(surf, (0,170,230),
                            (cx-8, cy-body_h-3, 16, 10))
        pygame.draw.ellipse(surf, (180,230,255),
                            (cx-4, cy-body_h-2,  8,  6))

        led_phase = int(self.t * 6) % 2
        led_c = LED_COL if led_phase == 0 else (0,100,80)
        pygame.draw.circle(surf, led_c, (cx, cy-body_h+1), 3)

        for dx, dy, name in SENSOR_DIRS:
            dist = self.sensors[name]
            ipx, ipy = iso(self.wx + dx*dist, self.wy + dy*dist, DRONE_H+self.bob)
            sc = RED_COL if dist<=1 else AMBER if dist<=3 else (0,150,80)
            pygame.draw.line(surf, sc, (cx,cy), (ipx,ipy), 1)
            if dist < SENSOR_RANGE:
                pygame.draw.circle(surf, sc, (ipx,ipy), 3)

        ht = font_xs.render(f"ALT:{int(DRONE_H)}m", True, ACCENT)
        surf.blit(ht, (cx+14, cy-16))

def gen_obstacles(dp, target=None):
    obs = set()
    excl = set()
    for dx in range(-3,4):
        for dy in range(-3,4):
            excl.add((dp[0]+dx, dp[1]+dy))
    if target:
        for dx in range(-2,3):
            for dy in range(-2,3):
                excl.add((target[0]+dx, target[1]+dy))

    for _ in range(8):
        ox=random.randint(2,COLS-6); oy=random.randint(2,ROWS-6)
        w=random.randint(2,4);       h=random.randint(2,4)
        for i in range(w):
            for j in range(h):
                c=(ox+i,oy+j)
                if c not in excl and 0<=c[0]<COLS and 0<=c[1]<ROWS:
                    obs.add(c)
  
    for _ in range(5):
        sx=random.randint(2,COLS-9); sy=random.randint(2,ROWS-4)
        l=random.randint(4,8); hz=random.choice([True,False])
        for i in range(l):
            c=(sx+i,sy) if hz else (sx,sy+i)
            if c not in excl and 0<=c[0]<COLS and 0<=c[1]<ROWS:
                obs.add(c)
   
    for _ in range(10):
        c=(random.randint(1,COLS-2),random.randint(1,ROWS-2))
        if c not in excl: obs.add(c)
    return obs

def gen_block_heights(obstacles):
    """Cdo bllok ka lartesi te rastesishme (por konstante)."""
    heights = {}
    for cell in obstacles:
        random.seed(cell[0]*100 + cell[1])
        heights[cell] = random.randint(20, 60)
    return heights

def gen_block_variations(obstacles):
    variations = {}
    for cell in obstacles:
        random.seed(cell[0]*7 + cell[1]*13)
        variations[cell] = random.randint(0, 3)
    return variations

class HUD:
    def __init__(self):
        self.log = []
        self.radar_a = 0.0
        self.font_xs = pygame.font.SysFont("Consolas", 10)
        self.font_sm = pygame.font.SysFont("Consolas", 12)
        self.font_md = pygame.font.SysFont("Consolas", 14, bold=True)
        self.font_lg = pygame.font.SysFont("Consolas", 18, bold=True)
        self._surf_cache = {}

    def add_log(self, msg, kind="info"):
        ts = time.strftime("%H:%M:%S")
        self.log.append((ts, msg[:28], kind))
        if len(self.log) > 12: self.log.pop(0)

    def draw(self, screen, drone, target, t, status, HUD_X, HUD_W):
        self.radar_a = (self.radar_a + 2.0) % 360
        W = HUD_W - 16

        pygame.draw.rect(screen, HUD_BG, (HUD_X, 0, HUD_W, WIN_H))
        pygame.draw.line(screen, HUD_BORDER, (HUD_X,0),(HUD_X,WIN_H), 2)

        y = 10

        pygame.draw.line(screen, ACCENT, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=6
        t1 = self.font_lg.render("DRONE NAV 3D", True, ACCENT)
        screen.blit(t1, (HUD_X+(HUD_W-t1.get_width())//2, y)); y+=22
        t2 = self.font_xs.render("AUTONOMOUS SYSTEM  v3.0", True, DIM)
        screen.blit(t2, (HUD_X+(HUD_W-t2.get_width())//2, y)); y+=16
        pygame.draw.line(screen, ACCENT, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=8

        rc = (HUD_X + HUD_W//2, y+52); rr=48
        self._radar(screen, drone, rc, rr)
        y += 114
        pygame.draw.line(screen, DIM, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=8

        screen.blit(self.font_md.render("◈ TELEMETRI", True, ACCENT),(HUD_X+8,y)); y+=16
        bat=drone.battery
        bc=GREEN if bat>50 else AMBER if bat>20 else RED_COL
        self._bar(screen,HUD_X+8,y,W,"BATERIA",bat/100,bc,f"{bat:.0f}%"); y+=20
        spd=drone.velocity
        self._bar(screen,HUD_X+8,y,W,"SHPEJTESIA",min(1,spd/20),ACCENT,f"{spd:.1f}km/h"); y+=20
        self._bar(screen,HUD_X+8,y,W,"NXEHTESIA",drone.heat,AMBER,f"{int(drone.heat*60+25)}C"); y+=20
        pygame.draw.line(screen, DIM, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=8

        screen.blit(self.font_md.render("◈ POZICIONI", True, ACCENT),(HUD_X+8,y)); y+=16
        rows=[
            ("POS",   f"({drone.gx},{drone.gy})"),
            ("DEST",  f"({target[0]},{target[1]})" if target else "—"),
            ("HAPAT", str(drone.steps)),
            ("MBETUR",f"{max(0,len(drone.path)-drone.pidx)}"),
        ]
        for lbl,val in rows:
            screen.blit(self.font_sm.render(lbl,True,DIM),(HUD_X+8,y))
            vt=self.font_sm.render(val,True,WHITE)
            screen.blit(vt,(HUD_X+HUD_W-vt.get_width()-8,y)); y+=15
        pygame.draw.line(screen, DIM, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=8

        screen.blit(self.font_md.render("◈ SENSORET", True, ACCENT),(HUD_X+8,y)); y+=16
        for i,(dx,dy,name) in enumerate(SENSOR_DIRS):
            dist=drone.sensors.get(name,SENSOR_RANGE)
            sc=GREEN if dist>3 else AMBER if dist>1 else RED_COL
            col_x=HUD_X+8+(i%2)*(W//2+4)
            ry2=y+(i//2)*17
            screen.blit(self.font_xs.render(f"{name:<3}",True,DIM),(col_x,ry2))
            bx=col_x+28; bw=(W//2)-34
            pygame.draw.rect(screen,(15,25,50),(bx,ry2+3,bw,7),border_radius=3)
            fw=int(bw*dist/SENSOR_RANGE)
            if fw>0: pygame.draw.rect(screen,sc,(bx,ry2+3,fw,7),border_radius=3)
            sv=self.font_xs.render(f"{dist}m",True,sc)
            screen.blit(sv,(bx+bw+2,ry2+1))
        y+=(len(SENSOR_DIRS)//2)*17+4
        pygame.draw.line(screen, DIM, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=8

        if drone.flying:   st,sc2="● FLUTURON",GREEN
        elif drone.reached:st,sc2="✓ ARRITI",ACCENT
        else:              st,sc2="○ PRITJE",DIM
        screen.blit(self.font_md.render(st,True,sc2),(HUD_X+8,y)); y+=20
        pygame.draw.line(screen, DIM, (HUD_X+8,y),(HUD_X+HUD_W-8,y),1); y+=6

        screen.blit(self.font_md.render("◈ LOG", True, ACCENT),(HUD_X+8,y)); y+=14
        for ts,msg,kind in self.log[-7:]:
            lc=GREEN if kind=="ok" else AMBER if kind=="warn" else RED_COL if kind=="err" else DIM
            screen.blit(self.font_xs.render(ts,True,DIM),(HUD_X+8,y))
            screen.blit(self.font_xs.render(msg,True,lc),(HUD_X+68,y)); y+=13

        pygame.draw.line(screen, ACCENT, (HUD_X+8,WIN_H-20),(HUD_X+HUD_W-8,WIN_H-20),1)
        screen.blit(self.font_xs.render(status[:36],True,DIM),(HUD_X+8,WIN_H-14))

    def _bar(self,surf,x,y,w,label,frac,col,valstr):
        surf.blit(self.font_xs.render(label,True,DIM),(x,y))
        by=y+11; bw=w
        pygame.draw.rect(surf,(15,25,50),(x,by,bw,7),border_radius=3)
        fw=max(0,int(bw*min(1,frac)))
        if fw>0: pygame.draw.rect(surf,col,(x,by,fw,7),border_radius=3)
        vt=self.font_xs.render(valstr,True,col)
        surf.blit(vt,(x+bw-vt.get_width(),y))

    def _radar(self,surf,drone,center,radius):
        cx,cy=center
        for r in range(radius,0,-12):
            pygame.draw.circle(surf,GRID_COL,(cx,cy),r,1)
        pygame.draw.line(surf,GRID_COL,(cx-radius,cy),(cx+radius,cy),1)
        pygame.draw.line(surf,GRID_COL,(cx,cy-radius),(cx,cy+radius),1)
       
        ra=math.radians(self.radar_a)
        ex=int(cx+radius*math.cos(ra)); ey=int(cy+radius*math.sin(ra))
        pygame.draw.line(surf,(0,200,160),(cx,cy),(ex,ey),2)
        
        for i in range(1,30):
            ra2=math.radians(self.radar_a-i*2)
            a2=max(0,60-i*2)
            if a2<5: break
            ex2=int(cx+radius*math.cos(ra2)); ey2=int(cy+radius*math.sin(ra2))
            c=max(0,a2)
            pygame.draw.line(surf,(0,c,c//2),(cx,cy),(ex2,ey2),1)
       
        scale=radius/max(COLS,ROWS)*1.5
        for dx,dy,name in SENSOR_DIRS:
            dist=drone.sensors[name]
            if dist<SENSOR_RANGE:
                ox=int(cx+dx*dist*scale*1.0)
                oy=int(cy+dy*dist*scale*0.5)
                sc=RED_COL if dist<=1 else AMBER if dist<=3 else GREEN
                pygame.draw.circle(surf,sc,(ox,oy),3)
        pygame.draw.circle(surf,DRONE_TOP,(cx,cy),4)
        rt=self.font_xs.render("RADAR",True,DIM)
        surf.blit(rt,(cx-rt.get_width()//2,cy+radius+4))


def draw_scene(surf, obstacles, block_h, block_var, target, drone, t, font_xs):
   
    surf.fill(SKY_TOP)
    h_mid = WIN_H // 2
    pygame.draw.rect(surf, SKY_BOT, (0, h_mid, WIN_W, WIN_H - h_mid))

    all_tiles = [(gx+gy, gx, gy) for gx in range(COLS) for gy in range(ROWS)]
    all_tiles.sort()

    for _, gx, gy in all_tiles:
        cell = (gx, gy)
        if cell in obstacles:
            bh  = block_h.get(cell, BLOCK_H)
            bv  = block_var.get(cell, 0)
            draw_block(surf, gx, gy, bh, bv)
        else:
           
            cx, cy = iso(gx, gy)
            is_even = (gx+gy)%2==0
            tc = (FLOOR_TOP[0]+(is_even*4), FLOOR_TOP[1]+(is_even*4), FLOOR_TOP[2]+(is_even*4))
            draw_tile(surf, gx, gy, tc, FLOOR_L, FLOOR_R, 0, GRID_COL)

        if target and cell == target:
            tx, ty = iso(gx, gy)
            pulse = 0.5 + 0.5*math.sin(t*5)
            arm   = int(10 + pulse*4)
            pygame.draw.line(surf, TARGET_COL,(tx-arm,ty),(tx+arm,ty),3)
            pygame.draw.line(surf, TARGET_COL,(tx,ty-arm//2),(tx,ty+arm//2),3)
            pygame.draw.circle(surf, TARGET_COL,(tx,ty),5,2)
            
            for deg in range(0,360,90):
                ra  = math.radians(deg + t*120)
                r2  = int(18+pulse*4)
                rpx = int(tx + r2*math.cos(ra)*0.9)
                rpy = int(ty + r2*math.sin(ra)*0.45)
                pygame.draw.circle(surf, TARGET_COL,(rpx,rpy),3)
           
            lt = font_xs.render("TARGET", True, TARGET_COL)
            surf.blit(lt,(tx-lt.get_width()//2, ty-24))

    drone.draw(surf, font_xs)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Autonomous Drone Navigation  —  3D Isometric v3.0")
    clock  = pygame.time.Clock()

    font_xs = pygame.font.SysFont("Consolas", 10)

    HUD_X   = WIN_W - 240
    HUD_WID = 240

    DRONE_START = (3, 3)
    drone       = Drone(*DRONE_START)
    obstacles   = gen_obstacles(DRONE_START)
    block_h     = gen_block_heights(obstacles)
    block_var   = gen_block_variations(obstacles)
    target      = None
    hud         = HUD()
    status      = "DJATHTE:vendos dest | MAJTE:bllok | SPACE:nis | R:reset | G:pengesa"

    hud.add_log("System online", "ok")
    hud.add_log("Set target (RMB)", "info")

    scene_surf = pygame.Surface((HUD_X, WIN_H))
    t = 0.0
    dt = 1.0 / FPS

    running = True
    while running:
        dt_real = clock.tick(FPS) / 1000.0
        dt_real = min(dt_real, 0.05)   
        t += dt_real

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT: running = False

            elif ev.type == pygame.MOUSEBUTTONDOWN:
                mx, my = ev.pos
                if mx >= HUD_X: continue
                gx, gy = iso_inv(mx, my)
                cell = (gx, gy)
                if not (0<=gx<COLS and 0<=gy<ROWS): continue

                if ev.button == 3:
                    if cell in obstacles:
                        hud.add_log("Cell blocked!", "err")
                    elif cell==(drone.gx,drone.gy):
                        hud.add_log("Drone is here!", "warn")
                    else:
                        target = cell
                        drone.flying=False; drone.reached=False
                        hud.add_log(f"Target:{cell}","ok")
                        status="SPACE per te nisur."

                elif ev.button==1 and not drone.flying:
                    if cell==(drone.gx,drone.gy): pass
                    elif cell in obstacles:
                        obstacles.discard(cell)
                        block_h.pop(cell,None); block_var.pop(cell,None)
                        hud.add_log(f"Removed {cell}","warn")
                    else:
                        obstacles.add(cell)
                        random.seed(cell[0]*100+cell[1])
                        block_h[cell]=random.randint(20,55)
                        block_var[cell]=random.randint(0,3)
                        hud.add_log(f"Added {cell}","warn")

            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False

                elif ev.key == pygame.K_SPACE:
                    if drone.flying:
                        drone.flying=False
                        hud.add_log("Paused.","warn")
                    elif target:
                        p=astar((drone.gx,drone.gy),target,obstacles)
                        if p:
                            drone.set_path(p)
                            drone.trail=[]
                            hud.add_log(f"Route:{len(p)} steps","ok")
                            status=f"Navigating — {len(p)} steps."
                        else:
                            hud.add_log("No path!","err")
                            status="No path! Remove obstacles."
                    else:
                        hud.add_log("Set target first!","warn")

                elif ev.key==pygame.K_r:
                    drone=Drone(*DRONE_START)
                    obstacles=gen_obstacles(DRONE_START)
                    block_h=gen_block_heights(obstacles)
                    block_var=gen_block_variations(obstacles)
                    target=None
                    hud.add_log("Reset.","warn")
                    status="Reset. Set a new target."

                elif ev.key==pygame.K_g and not drone.flying:
                    obstacles=gen_obstacles((drone.gx,drone.gy),target)
                    block_h=gen_block_heights(obstacles)
                    block_var=gen_block_variations(obstacles)
                    hud.add_log("New obstacles.","warn")

        drone.update(obstacles, dt_real)

        if drone.reached:
            hud.add_log(f"Arrived! Bat:{drone.battery:.0f}%","ok")
            status="Target reached! Set new destination."
            drone.reached=False; target=None

        draw_scene(scene_surf, obstacles, block_h, block_var,
                   target, drone, t, font_xs)
        screen.blit(scene_surf, (0,0))
        hud.draw(screen, drone, target, t, status, HUD_X, HUD_WID)

        fps_r = clock.get_fps()
        pygame.display.set_caption(
            f"Drone Nav 3D  |  ({drone.gx},{drone.gy})  |  "
            f"Bat:{drone.battery:.0f}%  |  FPS:{fps_r:.0f}")

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()