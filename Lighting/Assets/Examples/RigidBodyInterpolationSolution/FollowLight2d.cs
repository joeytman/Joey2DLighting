using UnityEngine;
using System.Collections;

public class FollowLight2d : MonoBehaviour {

	public GameObject toFollow;
	
	// Update is called once per frame
	void Update () {
		gameObject.transform.position = toFollow.transform.position;
	}
}
