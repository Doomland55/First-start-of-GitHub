using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// взято с youtube. Автор: Sebastian Lague
public class PoolObject : MonoBehaviour {

	// для подключения этой функции к обьекту пула нужно в скрипте (в любом, висящем на обьекте пула), где будем производить
	// манипуляции, сделать наследование не от MonoBehaviour, а от PoolObject и прописать функцию 
	// public override void OnObjectReuse(object[] par) { тело функции } 
	//	Например 
	//	public override void OnObjectReuse(object[] par){
	//		this.speed = (int)par[0];
	//		this.BallisticFactor =  (float) par[1];
	//		this.windVelo = (Vector3)par[2];
	//	}
		
	public virtual void OnObjectReuse(object[] _params) {}

		

	public void ReturnToPool() {
		gameObject.SetActive (false);
	}
}
