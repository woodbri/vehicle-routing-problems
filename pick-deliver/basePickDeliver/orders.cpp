#include "orders.h"

    Orders::Orders() {
       orders.clear();
       fifo.clear();
       lifo.clear();
       push.clear();
    };


    Orders::Orders(BucketN &nodes,const Dpnode &depot) {
       orders.clear();
       fifo.clear();
       lifo.clear();
       push.clear();
       makeOrders(nodes,depot);
    };


    void Orders::makeOrders (BucketN &nodes,const Dpnode &depot)
    {
      orders.clear();
      int oid = 0;
      int j;
      Order order;
      // for each pickup, get its delivery and create an order
      for (int i=0; i<nodes.size(); i++) {
        if (nodes[i].isdepot() or nodes[i].isdelivery()) continue;
           for (j=0; j<nodes.size() and nodes[j].getnid()!=nodes[i].getdid(); j++) {}
           order.fillOrder(nodes[i],nodes[j],oid++,depot);
           orders.push_back(order);
      }
    }


void Orders::join(const Orders &other){
     orders.insert(orders.end(),other.orders.begin(),other.orders.end());
};


bool Orders::isIncompatibleOrder(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= orderx.getoid();
    return not compat[oidx][oidy];
}

void Orders::setCompatibility( const Compatible &twc)  {
    int Px,Dx,Py,Dy;
std::cout<<"set Compatibility \n";
    compat.resize(size());
    fifo.resize(size());
    lifo.resize(size());
    push.resize(size());
    for (int i=0;i<size();i++) {
        compat[i].resize(orders.size());
        fifo[i].resize(orders.size());
        lifo[i].resize(orders.size());
        push[i].resize(orders.size());
    }
    for (int x=0;x<size();x++) {
        for (int y=x;y<size(); y++) {
            Px= orders[x].getpid();
            Dx= orders[x].getdid();
            Py= orders[y].getpid();
            Dy= orders[y].getdid();
            if (x==y) { //check Pi -> Di
                compat[x][x] = (twc.isCompatibleIJ(Px,Dx))?true:false;
                push[x][x]=lifo[x][x]=fifo[x][x]=false;
            } else if (twc.isCompatibleIJ(Px,Dx) and twc.isCompatibleIJ(Py,Dy) ) {  //blue lines
                fifo[x][y]= twc.isCompatibleIJ(Px,Py) and twc.isCompatibleIJ(Py,Dx) and twc.isCompatibleIJ(Dx,Dy); 
                lifo[x][y]= twc.isCompatibleIJ(Px,Py) and twc.isCompatibleIJ(Py,Dx) and twc.isCompatibleIJ(Dy,Dx); 
                push[x][y]= twc.isCompatibleIJ(Px,Py) and twc.isCompatibleIJ(Dx,Py) and twc.isCompatibleIJ(Dx,Dy); 
                compat[x][y]=lifo[x][y] or fifo[x][y] or push[x][y];

                fifo[y][x]= twc.isCompatibleIJ(Py,Px) and twc.isCompatibleIJ(Px,Dy) and twc.isCompatibleIJ(Dy,Dx); 
                lifo[y][x]= twc.isCompatibleIJ(Py,Px) and twc.isCompatibleIJ(Px,Dy) and twc.isCompatibleIJ(Dx,Dy); 
                push[y][x]= twc.isCompatibleIJ(Py,Px) and twc.isCompatibleIJ(Dy,Px) and twc.isCompatibleIJ(Dy,Dx); 
                compat[y][x]=lifo[y][x] or fifo[y][x] or push[y][x];
            } else  compat[x][y]=lifo[x][y] = fifo[x][y] = push[x][y] = compat[y][x]=lifo[y][x] = fifo[y][x] = push[y][x] =false;
         }
     }
}


/*
    void Orders::erase (int fromPos, int toPos) { 
         if (fromPos==toPos) orders.erase(fromPos); //[fromPos]
         else if (fromPos<toPos) orders.erase(path.begin()+fromPos,path.begin()+toPos); //[fromPos,toPos)
         else  orders.erase(path.begin()+toPos,path.begin()+fromPos); //[toPos,fromPos)
    };
*/
    void Orders::dump() const {
        std::cout << "Orders: "; 
        const_iterator it = orders.begin();
        for (const_iterator it = orders.begin(); it != orders.end();it++)
            std::cout << "(" << it->getpid()<<","<<it->getdid()<<") \t";
        std::cout << std::endl;
    };

    void Orders::dumpCompat() const {
        for (int i=0;i<4;i++) dumpCompats(i);
    } 

    void Orders::dumpCompats(int kind) const {
        if (kind>3) return;
        switch (kind) {
                    case 0: std::cout << "\n\n COMPAT\t ";  break;
                    case 1: std::cout << "\n\n FIFO\t ";  break;
                    case 2: std::cout << "\n\n LIFO\t ";  break;
                    case 3: std::cout << "\n\n PUSH\t ";  break;
        }
        const_iterator it = orders.begin();
        for (const_iterator it = orders.begin(); it != orders.end();it++)
            std::cout << "(" << it->getpid()<<","<<it->getdid()<<") \t";
        int i=0;
        for (int i=0;i<size();i++) {
           std::cout << "\n(" << orders[i].getpid()<<","<<orders[i].getdid()<<") \t";
           for (int j=0;j<size();j++) 
                switch (kind) {
                    case 0: std::cout << (compat[i][j]?"YES":"no") <<"\t"; break;
                    case 1: std::cout << (fifo[i][j]?"YES":"no") <<"\t"; break;
                    case 2: std::cout << (lifo[i][j]?"YES":"no") <<"\t"; break;
                    case 3: std::cout << (push[i][j]?"YES":"no") <<"\t"; break;
                }
        }
    }
    
