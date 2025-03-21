#ifndef C_2D_TABLE_H
#define C_2D_TABLE_H


class c_2d_table
{
public:
    c_2d_table(int sr, int sc);
    int* data_ptr;
    void setValue(int row, int col, int value);
private:
    int rows,cols;

};

#endif // C_2D_TABLE_H
