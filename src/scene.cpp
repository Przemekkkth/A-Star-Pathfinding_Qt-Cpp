#include "scene.h"
#include <algorithm>
#include <QDebug>
#include <QKeyEvent>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsLineItem>

Scene::Scene(QObject *parent)
    : QGraphicsScene(parent)
{
    setSceneRect(0,0, SCENE_SIZE.width(), SCENE_SIZE.height());
    OnCreateScene();
    connect(&m_timer, &QTimer::timeout, this, &Scene::loop);
    m_timer.start(m_loopTime);
    m_elapsedTimer.start();
}

void Scene::OnCreateScene()
{
    nodes = new Node[nMapWidth * nMapHeight];
    for(int x = 0; x < nMapWidth; ++x)
    {
        for(int y = 0; y < nMapHeight; ++y)
        {
            nodes[y * nMapWidth + x].x = x;
            nodes[y * nMapWidth + x].y = y;
            nodes[y * nMapWidth + x].bObstacle = false;
            nodes[y * nMapWidth + x].parent = nullptr;
            nodes[y * nMapWidth + x].bVisited = false;
        }
    }

    for(int x = 0; x < nMapWidth; ++x)
    {
        for(int y = 0; y < nMapHeight; ++y)
        {
            if(y>0)
            {
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
            }
            if(y<nMapHeight-1)
            {
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
            }
            if (x>0)
            {
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
            }
            if(x<nMapWidth-1)
            {
                nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);
            }
        }
    }

    nodeStart = &nodes[(nMapHeight / 2) * nMapHeight + 1];
    nodeEnd   = &nodes[(nMapHeight / 2) * nMapWidth + nMapWidth-2];
}

void Scene::OnUpdateScene()
{
    int nNodeSize   = 9 * UNIT_SIZE.width();
    int nNodeBorder = 9 * UNIT_SIZE.width();
    int nSelectedNodeX = m_mousePosX / nNodeSize;
    int nSelectedNodeY = m_mousePosY / nNodeSize;
    if(m_mouseReleased)
    {
        if(nSelectedNodeX >= 0 && nSelectedNodeX < nMapWidth)
        {
            if(nSelectedNodeY >= 0 && nSelectedNodeY < nMapHeight)
            {
                if (m_shiftPressed)
                {
                    nodeStart = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
                }
                else if (m_controlPressed)
                {
                    nodeEnd = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
                }
                else
                {
                    nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle = !nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle;
                }

                solveAStar(); // Solve in "real-time" gives a nice effect
                m_mouseReleased = false;
            }
        }
    }
    ////drawing
    clear();
    drawConnections(nNodeSize);
}

bool Scene::solveAStar()
{
    for(int x = 0; x < nMapWidth; ++x)
    {
        for(int y = 0; y < nMapHeight; ++y)
        {
            nodes[y*nMapWidth + x].bVisited    = false;
            nodes[y*nMapWidth + x].fGlobalGoal = INFINITY;
            nodes[y*nMapWidth + x].fLocalGoal  = INFINITY;
            nodes[y*nMapWidth + x].parent      = nullptr;
        }
    }

    auto distance = [](Node* a, Node* b)
    {
        return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    };

    auto heuristic = [distance](Node* a, Node* b)
    {
        return distance(a, b);
    };

    Node* nodeCurrent = nodeStart;
    nodeStart->fLocalGoal = 0.0f;
    nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

    QList<Node*> listNotTestedNodes;
    listNotTestedNodes.push_back(nodeStart);

    while(!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)
    {
        std::sort(listNotTestedNodes.begin(), listNotTestedNodes.end(), [](const Node* lhs, const Node* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal;});

        while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
        {
            listNotTestedNodes.pop_front();
        }

        if(listNotTestedNodes.empty())
            break;

        nodeCurrent = listNotTestedNodes.front();
        nodeCurrent->bVisited = true;

        for(auto nodeNeighbour : nodeCurrent->vecNeighbours)
        {
            if(!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
            {
                listNotTestedNodes.push_back(nodeNeighbour);
            }

            float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

            if(fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
            {
                nodeNeighbour->parent = nodeCurrent;
                nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;
                nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
            }
        }
    }

    return true;
}

void Scene::drawConnections(int nodeSize)
{
    for (int x = 0; x < nMapWidth; x++)
    {
        for (int y = 0; y < nMapHeight; y++)
        {
            for (auto n : nodes[y*nMapWidth + x].vecNeighbours)
            {
                QGraphicsLineItem* lineItem = new QGraphicsLineItem();
                QPen pen((QColor(Qt::darkBlue)));
                pen.setWidth(UNIT_SIZE.width());
                pen.setBrush(QBrush(QColor(Qt::darkBlue)));
                lineItem->setPen(pen);
                lineItem->setLine(x*nodeSize + nodeSize / 2, y*nodeSize + nodeSize / 2,
                                  n->x*nodeSize + nodeSize / 2, n->y*nodeSize + nodeSize / 2);
                addItem(lineItem);
            }
        }
    }
}

void Scene::loop()
{
    m_deltaTime = m_elapsedTimer.elapsed();
    m_elapsedTimer.restart();

    m_loopTime += m_deltaTime;
    if( m_loopTime > m_loopSpeed)
    {
        m_loopTime -= m_loopSpeed;
        OnUpdateScene();
    }
}

void Scene::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Shift)
    {
        m_shiftPressed = true;
    }
    else if(event->key() == Qt::Key_Control)
    {
        m_controlPressed = true;
    }

    QGraphicsScene::keyPressEvent(event);
}

void Scene::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Shift)
    {
        m_shiftPressed = false;
    }
    else if(event->key() == Qt::Key_Control)
    {
        m_controlPressed = false;
    }
    QGraphicsScene::keyReleaseEvent(event);
}

void Scene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    m_mouseReleased = false;
    QGraphicsScene::mousePressEvent(event);
}

void Scene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    m_mousePosX = event->scenePos().x();
    m_mousePosY = event->scenePos().y();
    m_mouseReleased = true;
    qDebug() << "pos " << m_mousePosX << " " << m_mousePosY;


    QGraphicsScene::mouseReleaseEvent(event);
}
