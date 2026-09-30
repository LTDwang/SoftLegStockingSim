using UnityEngine;

public struct StockingParticle
{
    //当前物理位置
    public Vector3 position;
    //上一帧位置
    public Vector3 previousPosition;
    //该点是否固定
    public bool isFixed;

    public StockingParticle(Vector3 position)
    {
        this.position = position;
        this.previousPosition = position;
        this.isFixed = false;
    }
}