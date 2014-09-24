#include "orders.h"

    Orders::Orders() {
       orders.clear();
       fifo.clear();
       lifo.clear();
       push.clear();
    };


    Orders::Orders(Bucket &nodes,const Dpnode &depot) {
       orders.clear();
       fifo.clear();
       lifo.clear();
       push.clear();
       makeOrders(nodes,depot);
    };

// manipulation
   void Orders::removeIncompatible(const Order &from, Orders &incompat) {
          for (int i=0;i<orders.size();i++) {
              if (isINCOMPAT(from,orders[i])) {
                  incompat.push_back(orders[i]);
                  orders.erase(orders.begin()+i);
                  i--;
              };
          }
   }


   void Orders::removeOrder(int oid) {
          for (int i=0;i<orders.size();i++) {
              if (orders[i].getoid()==oid) {
                  orders.erase(orders.begin()+i);
                  break;
              };
          }
   }



   void Orders::removeOrder(const Order &order) {
       removeOrder(order.getoid());
   }



   int Orders::leastReachable() const { //retruns position
//std::cout << "Enter Least Reachable\n";
         int bestCount=size()+1;
         int count;
         int leastPos=-1;
         for (int j=0;j<orders.size();j++) {
           count=0;
//orders[j].dump();
//std::cout << "Enter Least Reachable 1--------> "<<j<<" \t" <<count<<" \t" <<bestCount<<" \n";
           for (int i=0;i<orders.size();i++) {
//orders[i].dump();
//std::cout << "Enter Least Reachable 1.1--------> "<<i<<" \t" <<count<<"\t"<<(isCOMPAT(orders[i],orders[j])?"YES":"no")<<" \n";
              count+=isCOMPAT(orders[i],orders[j])?1:0;
//std::cout << "Enter Least Reachable 1.2--------> "<<i<<" \t" <<count<<" \n";
           }
           if (count<bestCount) {
//std::cout << "Enter Least Reachable 2---->"<<j<<" \n";
              bestCount=count; leastPos=j;
           }
//std::cout << "Enter Least Reachable 3--------> "<<leastPos<<" \n";
          }
//std::cout << "EXIT Least Reachable 3--------> "<<leastPos<<" \n";
orders[leastPos].dump();
         return leastPos;
   }

   int Orders::reachesMost() const {
         int bestCount=-1;
         int count;
         int mostPos=-1;
         for (int i=0;i<orders.size();i++) {
           count=0;
           for (int j=0;j<orders.size();j++)
              count=isINCOMPAT(orders[i],orders[j])?count++:count;
           if (count>bestCount) {
              bestCount=count; mostPos=i;
           }
          }
         return mostPos;
   }


    void Orders::makeOrders (Bucket &nodes,const Dpnode &depot)
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
     Order order;
     for (int i=0;i<other.size();i++) {
         order=other.orders[i];
         orders.push_back(order);
     }
};


bool Orders::isPUSHcompat(int oidx, int oidy) const { return  push[oidx][oidy]; }
bool Orders::isFIFOcompat(int oidx, int oidy) const { return  fifo[oidx][oidy]; }
bool Orders::isLIFOcompat(int oidx, int oidy) const { return  lifo[oidx][oidy]; }
bool Orders::isCOMPAT(int oidx, int oidy) const { return not compat[oidx][oidy]; }
bool Orders::isINCOMPAT(int oidx, int oidy) const { return not compat[oidx][oidy]; }

bool Orders::isPUSHcompat(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= ordery.getoid();
    return  push[oidx][oidy];
}

bool Orders::isFIFOcompat(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= ordery.getoid();
    return  fifo[oidx][oidy];
}

bool Orders::isLIFOcompat(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= ordery.getoid();
    return  lifo[oidx][oidy];
}

bool Orders::isCOMPAT(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= ordery.getoid();
    return  compat[oidx][oidy];
}

bool Orders::isINCOMPAT(const Order &orderx, const Order &ordery) const {
    int oidx,oidy;
    oidx= orderx.getoid();
    oidy= ordery.getoid();
    return not compat[oidx][oidy];
}

void Orders::setCompatibility( const TWC<Dpnode> &twc, Vehicle &v)  {
 
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
std::cout<<"("<<Px<<","<<Dx<<") ("<<Py<<","<<Dy<<")";
std::cout<<"("<<twc.node(Px).getnid()<<","<<twc.node(Dx).getnid()<<") ("<<twc.node(Py).getnid()<<","<<twc.node(Dy).getnid()<<")\n";
            if (x==y) { //check Pi -> Di
                compat[x][x] = (twc.isCompatibleIJ(Px,Dx))?true:false;
                push[x][x]=lifo[x][x]=fifo[x][x]=false;
            } else if (twc.isCompatibleIJ(Px,Dx) and twc.isCompatibleIJ(Py,Dy) ) {  //azul
                if (twc.isCompatibleIJ(Px,Py) and twc.isCompatibleIJ(Px,Dy)) {      //verde y violeta
                   fifo[x][y]= twc.isCompatibleIJ(Py,Dx) and twc.isCompatibleIJ(Dx,Dy) and v.e_feasable4(twc.node(Px),twc.node(Py),twc.node(Dx),twc.node(Dy)); 
                   lifo[x][y]= twc.isCompatibleIJ(Py,Dx) and twc.isCompatibleIJ(Dy,Dx) and v.e_feasable4(twc.node(Px),twc.node(Py),twc.node(Dy),twc.node(Dx));
                   push[x][y]= twc.isCompatibleIJ(Dx,Py) and twc.isCompatibleIJ(Dx,Dy) and v.e_feasable4(twc.node(Px),twc.node(Dx),twc.node(Py),twc.node(Dy)); 
                   compat[x][y]=lifo[x][y] or fifo[x][y] or push[x][y];
                } else compat[x][y]=lifo[x][y] = fifo[x][y] = push[x][y] = false;

                if (twc.isCompatibleIJ(Py,Px) and twc.isCompatibleIJ(Py,Dx)) {      //verde y violeta
                   fifo[y][x]= twc.isCompatibleIJ(Px,Dy) and twc.isCompatibleIJ(Dy,Dx) and v.e_feasable4(twc.node(Py),twc.node(Px),twc.node(Dy),twc.node(Dx)); 
                   lifo[y][x]= twc.isCompatibleIJ(Px,Py) and twc.isCompatibleIJ(Dx,Dy) and v.e_feasable4(twc.node(Py),twc.node(Px),twc.node(Dx),twc.node(Dy)); 
                   push[y][x]= twc.isCompatibleIJ(Dy,Px) and twc.isCompatibleIJ(Dy,Dx) and v.e_feasable4(twc.node(Py),twc.node(Dy),twc.node(Px),twc.node(Dx)); 
                   compat[y][x]=lifo[y][x] or fifo[y][x] or push[y][x];
                } else   compat[y][x]=lifo[y][x] = fifo[y][x] = push[y][x]=false;
            } else  compat[x][y]=lifo[x][y] = fifo[x][y] = push[x][y] = compat[y][x]=lifo[y][x] = fifo[y][x] = push[y][x] =false;
std::cout<<"fifo (X,Y) "<<(fifo[x][y]?"YES":"no")<<"\n";
std::cout<<"lifo (X,Y) "<<(lifo[x][y]?"YES":"no")<<"\n";
std::cout<<"push (X,Y) "<<(push[x][y]?"YES":"no")<<"\n";
std::cout<<"fifo (Y,X) "<<(fifo[y][x]?"YES":"no")<<"\n";
std::cout<<"lifo (Y,X) "<<(lifo[y][x]?"YES":"no")<<"\n";
std::cout<<"push (Y,X) "<<(push[y][x]?"YES":"no")<<"\n";
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
    
