using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
 class PoolItem{
	[HideInInspector]public string poolName;
	public GameObject poolPrefab;
	public int poolCount;
}


public class PoolManager : MonoBehaviour{

	[SerializeField]PoolItem[] poolItems;


	void Start(){
		Initialisation ();
	}
		

	void Initialisation(){
		for (int i =0; i < poolItems.Length; i++){
			if (poolItems [i].poolPrefab != null){
				poolItems [i].poolName = poolItems[i].poolPrefab.name;
				PoolManager.instance.CreatePool (poolItems[i].poolPrefab, poolItems[i].poolCount);
			}
		}
		Debug.Log ("Pool created successfully");
	}



	Dictionary<int,Queue<ObjectInstance>> poolDictionary = new Dictionary<int, Queue<ObjectInstance>> ();

	static PoolManager _instance;

	public static PoolManager instance {
		get {
			if (_instance == null) {
				_instance = FindObjectOfType<PoolManager> ();
			}
			return _instance;
		}
	}

	public void CreatePool(GameObject prefab, int poolSize) {
		int poolKey = prefab.GetInstanceID ();

		if (!poolDictionary.ContainsKey (poolKey)) {
			poolDictionary.Add (poolKey, new Queue<ObjectInstance> ());

			GameObject poolHolder = new GameObject (prefab.name + " pool");
			poolHolder.transform.parent = transform;

			for (int i = 0; i < poolSize; i++) {
				ObjectInstance newObject = new ObjectInstance(Instantiate (prefab) as GameObject);
				poolDictionary [poolKey].Enqueue (newObject);
				newObject.SetParent (poolHolder.transform);
			}
		}
	}

//	public void ReuseObject(GameObject prefab, Vector3 position, Quaternion rotation) {
//		int poolKey = prefab.GetInstanceID ();
//
//		if (poolDictionary.ContainsKey (poolKey)) {
//			ObjectInstance objectToReuse = poolDictionary [poolKey].Dequeue ();
//			poolDictionary [poolKey].Enqueue (objectToReuse);
//
//			objectToReuse.Reuse (position, rotation);
//		}
//	}

	public void ReuseObject(GameObject prefab, Vector3 position, Quaternion rotation, params object[] param) {
		int poolKey = prefab.GetInstanceID ();

		if (poolDictionary.ContainsKey (poolKey)) {
			ObjectInstance objectToReuse = poolDictionary [poolKey].Dequeue ();
			poolDictionary [poolKey].Enqueue (objectToReuse);

			objectToReuse.Reuse (position, rotation, param);
		}
	}

	public class ObjectInstance {

		GameObject gameObject;
		Transform transform;

		bool hasPoolObjectComponent;
		PoolObject poolObjectScript;

		public ObjectInstance(GameObject objectInstance) {
			gameObject = objectInstance;
			transform = gameObject.transform;
			gameObject.SetActive(false);

			if (gameObject.GetComponent<PoolObject>()) {
				hasPoolObjectComponent = true;
				poolObjectScript = gameObject.GetComponent<PoolObject>();
			}
		}

//		public void Reuse(Vector3 position, Quaternion rotation) {
//			if (hasPoolObjectComponent) {
//				poolObjectScript.OnObjectReuse ();
//			}
//
//			transform.position = position;
//			transform.rotation = rotation;
//			gameObject.SetActive (true);
//		}

		public void Reuse(Vector3 position, Quaternion rotation, object[] param) {
			transform.position = position;
			transform.rotation = rotation;

			if (hasPoolObjectComponent) {
				poolObjectScript.OnObjectReuse (param);
			}

			gameObject.SetActive (true);
		}

		public void SetParent(Transform parent) {
			transform.parent = parent;
		}
	}
}
