#include "c_2d_table.h"

c_2d_table::c_2d_table(int sr, int sc)
{
    data_ptr = new int[sr*sc];\
    rows=sr;
    cols=sc;
}
void c_2d_table::setValue(int row,int col,int value)
{
    int id = rows*row+col;
    data_ptr[id]=value;

}
