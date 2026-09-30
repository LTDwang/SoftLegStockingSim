public enum StockingConstraintType
{
    Circumferential,//环向
    Longitudinal,//纵向
    Shear,//斜向
    LongitudinalBend,//纵向隔一圈：防止整圈翻折
    CircumferentialBend//同圈隔一点：只抗压，防止粒子交叉堆叠
}

public struct StockingConstraint
{
    public StockingConstraintType type;//该两点之间所存在约束的种类
    public int particleA;
    public int particleB;
    public float originalLength;
    public float stiffness;//该约束的强度
    public StockingConstraint(StockingConstraintType type, int particleA, int particleB, float originalLength,float stiffness)
    {
        this.originalLength = originalLength;
        this.stiffness = stiffness;
        this.particleA = particleA;
        this.particleB = particleB;
        this.type = type;
    }
}