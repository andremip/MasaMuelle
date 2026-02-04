using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		Paused = true;
		TimeStep = 0.01f;
		Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
		IntegrationMethod = Integration.Explicit;
	}

	/// <summary>
	/// Integration method.
	/// </summary>
	public enum Integration
	{
		Explicit = 0,
		Symplectic = 1,
        Midpoint = 2,
        Verlet = 3,
        Implicit = 4,
	};

	#region InEditorVariables

	public bool Paused;
	public float TimeStep;
    public Vector3 Gravity;
    public List<GameObject> SimObjects;
    public Integration IntegrationMethod;

    #endregion

    #region OtherVariables
    private List<ISimulable> m_objs;
    private int m_numDoFs;
    private bool verletInit = true;
    VectorXD x_prev;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        //Parse the simulable objects and initialize their state indices
        m_numDoFs = 0;
        m_objs = new List<ISimulable>(SimObjects.Capacity);

        foreach (GameObject obj in SimObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();
            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable object
                simobj.Initialize(m_numDoFs, this);

                // Retrieve pos and vel size
                m_numDoFs += simobj.GetNumDoFs();
            }
        }
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;

    }

    public void FixedUpdate()
    {
        if (this.Paused)
            return; // Not simulating

        // Select integration method
        switch (this.IntegrationMethod)
        {
            case Integration.Explicit: this.stepExplicit(); break;
            case Integration.Symplectic: this.stepSymplectic(); break;
            case Integration.Midpoint: this.stepMidpoint(); break;
            case Integration.Verlet: this.stepVerlet(); break;
            case Integration.Implicit: this.stepImplicit(); break;
            default:
                throw new System.Exception("[ERROR] Should never happen!");
        }
    }

    #endregion

    /// <summary>
    /// Performs a simulation step using Explicit integration.
    /// </summary>
    private void stepExplicit()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += TimeStep * v;
        v += TimeStep * (Minv * f);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Symplectic integration.
    /// </summary>
    private void stepSymplectic()
	{
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);   // Inverse of the mass
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        // Fix the nodes that dont move
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        // TimeStep: symplectic euler
        v += TimeStep * (Minv * f);
        x += TimeStep * v;

        // Store the new values
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Midpoint integration.
    /// </summary>
    private void stepMidpoint()
    {
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD xMidPoint = new DenseVectorXD(m_numDoFs);
        VectorXD vMidPoint = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);   // Inverse of the mass
        Minv.Clear();

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        // Midpoint 1st step
        xMidPoint = x + v * TimeStep / 2;
        vMidPoint = v + (Minv * f) * TimeStep / 2;        

        // Use the x and v of the midpoint
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(xMidPoint);
            obj.SetVelocity(vMidPoint);
        }

        f.Clear();
        Minv.Clear();

        // Now the 2nd step
        foreach (ISimulable obj in m_objs)
        {
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x += vMidPoint * TimeStep;
        v += (Minv * f) * TimeStep;

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    private void verletInitFunc(VectorXD x, VectorXD v, VectorXD f, MatrixXD Minv)
    {
        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        // Fix the nodes that dont move
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        VectorXD x_firstH = new DenseVectorXD(m_numDoFs);
        x_firstH = x + v * TimeStep + 0.5 * (Minv * f) * TimeStep * TimeStep;

        x_prev = new DenseVectorXD(m_numDoFs);
        x_prev = x;

        // Store the new values
        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x_firstH);
            obj.SetVelocity(v);
        }
    }
    
    /// <summary>
    /// Performs a simulation step using Verlet integration.
    /// </summary>
    private void stepVerlet()
    {
        VectorXD x = new DenseVectorXD(m_numDoFs);
        VectorXD x_aux = new DenseVectorXD(m_numDoFs);
        VectorXD v = new DenseVectorXD(m_numDoFs);
        VectorXD f = new DenseVectorXD(m_numDoFs);
        f.Clear();
        MatrixXD Minv = new DenseMatrixXD(m_numDoFs);   // Inverse of the mass
        Minv.Clear();

        // Initialize verlet assuming constant acceleration
        if (verletInit)
        {
            verletInitFunc(x, v, f, Minv);
            verletInit = false;
        }

        foreach (ISimulable obj in m_objs)
        {
            obj.GetPosition(x);
            obj.GetVelocity(v);
            obj.GetForce(f);
            obj.GetMassInverse(Minv);
        }

        // Fix the nodes that dont move
        foreach (ISimulable obj in m_objs)
        {
            obj.FixVector(f);
            obj.FixMatrix(Minv);
        }

        x_aux = x;
        x = 2.0 * x - x_prev + (Minv * f) * TimeStep * TimeStep;
        x_prev = x_aux;

        v = (x - x_prev) / (2.0 * TimeStep);

        foreach (ISimulable obj in m_objs)
        {
            obj.SetPosition(x);
            obj.SetVelocity(v);
        }
    }

    /// <summary>
    /// Performs a simulation step using Implicit integration.
    /// </summary>
    private void stepImplicit()
    {
        // TO BE COMPLETED //
    }

}
