import time
import random
import drawSample
import math
import _tkinter
import sys
import imageToRects
import utils

#display = drawSample.SelectRect(imfile=im2Small,keepcontrol=0,quitLabel="")
args = utils.get_args()
visualize = utils.get_args()
drawInterval = 100 # 10 is good for normal real-time drawing

prompt_before_next=1  # ask before re-running sonce solved
SMALLSTEP = args.step_size # what our "local planner" can handle.
map_size,obstacles = imageToRects.imageToRects(args.world)
#Note the obstacles are the two corner points of a rectangle
#Each obstacle is (x1,y1), (x2,y2), making for 4 points
XMAX = map_size[0]
YMAX = map_size[1]

G = [  [ 0 ]  , [] ]   # nodes, edges
vertices = [ [args.start_pos_x, args.start_pos_y], [args.start_pos_x, args.start_pos_y + 10]]

# goal/target
tx = args.target_pos_x
ty = args.target_pos_y

# start
sigmax_for_randgen = XMAX/2.0
sigmay_for_randgen = YMAX/2.0
nodes=0
edges=1

def redraw(canvas):
    canvas.clear()
    canvas.markit( tx, ty, r=SMALLSTEP )
    drawGraph(G, canvas)
    for o in obstacles: canvas.showRect(o, outline='blue', fill='blue')
    canvas.delete("debug")


def drawGraph(G, canvas):
    global vertices,nodes,edges
    if not visualize: return
    for i in G[edges]:
       canvas.polyline(  [vertices[i[0]], vertices[i[1]] ]  )


def genPoint():
    if args.rrt_sampling_policy == "uniform":
        # Uniform distribution
        x = random.random()*XMAX
        y = random.random()*YMAX
    elif args.rrt_sampling_policy == "gaussian":
        # Gaussian with mean at the goal
        x = random.gauss(tx, sigmax_for_randgen)
        y = random.gauss(ty, sigmay_for_randgen)
    else:
        print "Not yet implemented"
        quit(1)

    bad = 1
    while bad:
        bad = 0
        if args.rrt_sampling_policy == "uniform":
            # Uniform distribution
            x = random.random()*XMAX
            y = random.random()*YMAX
        elif args.rrt_sampling_policy == "gaussian":
            # Gaussian with mean at the goal
            x = random.gauss(tx, sigmax_for_randgen)
            y = random.gauss(ty, sigmay_for_randgen)
        else:
            print "Not yet implemented"
            quit(1)
        # range check for gaussian
        if x<0: bad = 1
        if y<0: bad = 1
        if x>XMAX: bad = 1
        if y>YMAX: bad = 1
    return [x,y]

def returnParent(k, canvas):
    """ Return parent note for input node k. """
    for e in G[edges]:
        if e[1]==k:
            canvas.polyline(  [vertices[e[0]], vertices[e[1]] ], style=3  )
            return e[0]

def genvertex():
    vertices.append( genPoint() )
    return len(vertices)-1

def pointToVertex(p):
    vertices.append( p )
    return len(vertices)-1

def pickvertex():
    return random.choice( range(len(vertices) ))

def lineFromPoints(p1,p2):
    #TODO
    slope = (p2[1] - p1[1]) / (p2[0] - p1[0])
    # take average of y intercepts calculated from both points (averaging the accuracy)
    b1 = p1[1] - slope * p1[0]
    b2 = p2[1] - slope * p2[0]
    b = (b1 + b2) / 2
    return [slope, b]

def pointPointDistance(p1,p2):
    #TODO
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**0.5

def closestPointToPoint(G,p2):
    #TODO
    #return vertex index
    min = pointPointDistance(vertices[0], p2)
    index = 0
    for i in range(len(vertices)):
        if pointPointDistance(vertices[i], p2) < min:
            min = pointPointDistance(vertices[i], p2)
            index = i
    return index

def lineHitsRect(p1,p2,r):
    # TODO
    return not (max(p1[0], p2[0]) < r[0] or min(p1[0], p2[0]) > r[2] or max(p1[1], p2[1]) < r[1] or min(p1[1], p2[1]) > r[3])

def inRect(p,rect,dilation):
    """ Return 1 in p is inside rect, dilated by dilation (for edge cases). """
    #TODO
    return (rect[0]-dilation) <= p[0] <= (rect[2]+dilation) and (rect[1]-dilation) <= p[1] <= (rect[3]+dilation)

def inBounds(p):
    # helper function to determine if the point is inside the canvas
    return (0 <= p[0] <= XMAX) and (0 <= p[1] <= YMAX)

def rrt_search(G, tx, ty, canvas):

    global sigmax_for_randgen, sigmay_for_randgen
    n=0
    nsteps=0
    while 1:
        p = genPoint()
        v = closestPointToPoint(G,p)

        if not inBounds(p):
            continue

        if visualize:
            # if nsteps%500 == 0: redraw()  # erase generated points now and then or it gets too cluttered
            n=n+1
            if n>10:
                canvas.events()
                n=0

        toReject = False
        for o in obstacles:
            if lineHitsRect(vertices[v],p,o) or inRect(p,o,1):
                toReject = True

        if toReject:
            continue

        line = lineFromPoints(vertices[v], p)
        slope = line[0]
        intercept = line[1]

        angle = math.atan(slope)
        angle %= math.pi/2 # get angle between 0 and 90 degrees

        vx = vertices[v][0]
        vy = vertices[v][1]

        newPoint = [-1, -1]

        if vx > p[0]:
            newPoint[0] = vx-SMALLSTEP*math.cos(angle)
        else:
            newPoint[0] = vx+SMALLSTEP*math.cos(angle)
        if vy > p[1]:
            newPoint[1] = vy-SMALLSTEP*math.sin(angle)
        else:
            newPoint[1] = vy+SMALLSTEP*math.sin(angle)

        if not inBounds(newPoint):
            continue

        k = pointToVertex( newPoint )   # is the new vertex ID
        G[nodes].append(k)
        G[edges].append( (v,k) )
        nsteps = nsteps + 1

        if visualize:
            canvas.polyline(  [vertices[v], vertices[k] ]  )

        if pointPointDistance( newPoint, [tx,ty] ) < SMALLSTEP:
            print "Target achieved.", nsteps, "nodes in entire tree"
            if visualize:
                t = pointToVertex([tx, ty])  # is the new vertex ID
                G[edges].append((k, t))
                if visualize:
                    canvas.polyline([p, vertices[t]], 1)
                # while 1:
                #     # backtrace and show the solution ...
                #     canvas.events()
                nsteps = 0
                totaldist = 0
                while 1:
                    oldp = vertices[k]  # remember point to compute distance
                    k = returnParent(k, canvas)  # follow links back to root.
                    canvas.events()
                    if k <= 1: break  # have we arrived?
                    nsteps = nsteps + 1  # count steps
                    totaldist = totaldist + pointPointDistance(vertices[k], oldp)  # sum lengths
                print "Path length", totaldist, "using", nsteps, "nodes."

                global prompt_before_next
                if prompt_before_next:
                    canvas.events()
                    print "More [c,q,g,Y]>",
                    d = sys.stdin.readline().strip().lstrip()
                    print "[" + d + "]"
                    if d == "c": canvas.delete()
                    if d == "q": return
                    if d == "g": prompt_before_next = 0
                break

def main():
    #seed
    random.seed(args.seed)
    if visualize:
        canvas = drawSample.SelectRect(xmin=0,ymin=0,xmax=XMAX ,ymax=YMAX, nrects=0, keepcontrol=0)#, rescale=800/1800.)
        for o in obstacles: canvas.showRect(o, outline='red', fill='blue')
    while 1:
        # graph G
        redraw(canvas)
        G[edges].append( (0,1) )
        G[nodes].append(1)
        if visualize: canvas.markit( tx, ty, r=SMALLSTEP )

        drawGraph(G, canvas)
        rrt_search(G, tx, ty, canvas)

    if visualize:
        canvas.mainloop()

if __name__ == '__main__':
    main()
