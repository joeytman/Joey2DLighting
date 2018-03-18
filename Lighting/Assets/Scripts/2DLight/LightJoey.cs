
namespace PersonalDynamicLights
{
	using UnityEngine;
	using System.Collections;
	using System.Collections.Generic;

	/**
	 * The dynamic lighting parses the meshes in the world 
	 * and stores the useful information from them as type Vertex
	 */
	public class LightInteractVertex
	{
		public enum Location {MIDDLE, BEGINNING, END};

		public Location loc {get;set;} 
		public float angle {get;set;}
		public Vector3 pos {get;set;}
		public bool isExpectedPosition { get; set;}

	}


	public class LightJoey : MonoBehaviour {
		[Tooltip("The material overlaid on the area intended to be illuminated")]
		public Material lightMaterial;
		[Tooltip("Checking yes will increase performance in the game, but also make loading take longer")]
		public bool memoizeApproxTrigValsOnLoad;
		[Tooltip("This is the radius for how far in the game coordinates the light should raycast and extend to")]
		public float lightRadius = 20f;
		[Tooltip("The number of rays to cast")]
		[Range(4,20)]
		public int lightSegments = 8;
		[Tooltip("The layers for the lighting to interact with")]
		public LayerMask interactionLayers;
		[Tooltip("These layers will call the 'illuminated' function of the parent object of the light, passing in the rigidbody it collided with as its argument")]
		public LayerMask checkIlluminationLayers;

		private PolygonCollider2D[] allMeshes;
		private List<LightInteractVertex> allVertices;
		private Mesh lightMesh;


		// Called at startup of the scene
		void Start () {

			//Set in inspector
			if (memoizeApproxTrigValsOnLoad) {
				//loads lots of default vals for trig ops
				FastTrigCalculator.memoize (720);
			}

			allVertices = new List<LightInteractVertex>();

			//helper fxn that sets up lightMesh for rendering
			setupMeshFilter ();

		}
			
		/**
		 * Logic for calculating and updating dynamic light on every frame
		 * Five steps:
		 * 	1) Populate allMeshes with the PolygonCollider2Ds of everything in the scene within a certain
		 * 		radius of the current location of the light
		 * 	2) Go through all of the Colliders, find their points, and see which would be exposed to the light.
		 * 		Also, check to see if we need to cast light in places where there aren't polygons. 
		 * 		Also, take care of edge cases alongisde the corners of where light hits and doesn't hit
		 * 	3) Sort through every vertex, and manage conflicts that come from overlapping points before we expose it to rendering
		 * 	4) Use these vertices to actually generate the mesh we will overlap the lightMaterial onto
		 * 	5) Reset the bounds of the mesh so that we can update it again next frame
		 */
		void Update(){
			
			getAllMeshes();
			parseIntoVertices ();
			fixVertOrder ();
			renderLightMesh ();
			resetBounds ();

		}


		/**
		 * Step 1 of the lighting logic, called each frame
		 * Uses Physics2D.OverlapCircleAll to get each collider in the scene that's within
		 * the desired lighting radius
		 */
		void getAllMeshes(){
			Collider2D[] allColl2D = Physics2D.OverlapCircleAll(transform.position, lightRadius, interactionLayers);


			allMeshes = new PolygonCollider2D[allColl2D.Length];
			for (int i=0; i<allColl2D.Length; i++) {
				allMeshes[i] = (PolygonCollider2D) allColl2D[i];
			}
		}

		/**
		 * Step 2 of the lighting logic, called each frame
		 * Uses the list of meshes in radius we now have, parses this information into
		 * a list of instances of LightInteractVertex objects representing
		 * the information we need from each collider in range
		 * 
		 * These vertices are either: 
		 * 	1) the exact vertex that we overlapped with the circle
		 * 	2) some other point on a collider that intercepted the ray there
		 */
		void parseIntoVertices() {
			/**
			 * allVertices is populated each frame so we need to wipe it each time
			 * TODO: check to see what's quicker: 
			 * 		1) making a new list here, or 
			 * 		2) clearing the existing one and never allocating more memory
			 */
			allVertices.Clear ();

			List <LightInteractVertex> tempVerts = new List<LightInteractVertex>();

			//We want to go through every mesh in the light radius' range and see where on them the light hits
			for (int n = 0; n < allMeshes.Length; n++) {
				tempVerts.Clear();
				PolygonCollider2D colliderN = allMeshes[n];

				//This fxn raycasts at every point and creates vertex objects in the tempVerts list
				//It returns true if there's a possibility the collider stretches from Q1 to Q4, which breaks the
				//assumptions we can usually make about relative angles
				bool brokeBounds = raycastAtCollider (colliderN, tempVerts);

				//This should sort vertices from lowest to highest angles in the tempVerts list
				sortList(tempVerts); // sort first


				/**
				 * Now, things get complicated.
				 * 
				 * Two possibilities:
				 * 	1) We're lucky, and this collider doesn't wrap across coordinates Q1 to Q4
				 * 		We can proceed to assume lowest angle is sorted to bottom, highest on top
				 * 	2) We're unlucky, and it does wrap. We must iterate across points to determine which 
				 * 		points are the edges of the object
				 */
				int posLowAngle = 0;
				int posHighAngle = 0; 

				if (brokeBounds) { //We can't assume absolute ordering of angles if they wrap quadrants
					float lowestAngle = 3.15f;
					float highestAngle = -3.15f;
					for (int m = 0; m < tempVerts.Count; m++) {
						LightInteractVertex vert = tempVerts[m];
						float newRelAngle = (vert.angle > 3.14f) ? vert.angle - 6.28f : vert.angle; //This makes break point from Q2->Q3
						if (newRelAngle < lowestAngle) {
							lowestAngle = newRelAngle;
							posLowAngle = m;
						}
						if (newRelAngle > highestAngle) {
							highestAngle = newRelAngle;
							posHighAngle = m;
						}
					}
				} else {
					posHighAngle = tempVerts.Count - 1;
				}

				tempVerts[posLowAngle].loc = LightInteractVertex.Location.BEGINNING;
				tempVerts[posHighAngle].loc = LightInteractVertex.Location.END;
			
				allVertices.AddRange(tempVerts);

				/**
				 * At this point, we've added the vertices and their illumination points to the list
				 * All that we need to do is add vertices that we create directly at the edge of wherever the light is touching
				 * We already have identified the edges that touch the currently polygoncollider2d and which ones are 
				 * the furthest apart, so now we just need to search around that area
				 */

				//Do this twice, r=0 for BEGINNING, r=1 for END
				for(int r = 0; r<2; r++){
					Vector3 fromCast = new Vector3();
					bool isEndpoint = false;

					if (r == 0) {
							fromCast = transform.TransformPoint(tempVerts[posLowAngle].pos);
							isEndpoint = tempVerts[posLowAngle].isExpectedPosition;

						} else {
							fromCast = transform.TransformPoint(tempVerts[posHighAngle].pos);
							isEndpoint = tempVerts[posHighAngle].isExpectedPosition;
						}

						if(isEndpoint == true){
							Vector2 from = (Vector2)fromCast; //The world coords of where this edge point hit
							Vector2 dir = (from - (Vector2)transform.position); //the vector encoding direction from light to that vertex

							const float checkPointLastRayOffset= 0.005f; 

							from += (dir * checkPointLastRayOffset);
							//This basically fudges the direction very slightly in one direction

							RaycastHit2D rayCont = Physics2D.Raycast(from, dir, lightRadius, interactionLayers);
							Vector3 hitp;


							if(rayCont){
								//This means that fudging the edge very slightly made us hit another object which we want to shine light on
								hitp = rayCont.point;
							}else{
								//This means that when we fudge the edge slightly we instead hit nothing
								//Therefore we need to add a vertex lightRadius amount away
								Vector2 newDir = transform.InverseTransformDirection(dir);	
								//newDir needs to be relative to the light now, as we're not raycasting but instead just adding a vertex for the mesh from nothingness
								hitp = (Vector2)transform.TransformPoint( newDir.normalized * lightRadius);
								//hitp is this vector position that is floating lightradius units away from everything
							}

							Debug.DrawLine(fromCast, hitp, Color.green);

							LightInteractVertex vL = new LightInteractVertex();
							vL.pos = transform.InverseTransformPoint(hitp);

							vL.angle = FastTrigCalculator.getAngleTo(vL.pos.x, vL.pos.y, true);
							allVertices.Add(vL);
						}
					}
			}



			/**
			 * Finally, we have to consider the case where there aren't any colliders in directions but that we still need to cast light there.
			 * In that case, we can just get vertex locations at lightRadius distance from light, then raycast to there to check to make sure that
			 * there are no obstructions
			 */

			float theta = 0;
			float amount = (Mathf.PI * 2) / (float)lightSegments;

			for (int i = 0; i < lightSegments; i++)  {

				theta = amount * ((float)i);

				LightInteractVertex v = new LightInteractVertex();
				v.pos = new Vector3(FastTrigCalculator.SinRadApprox(theta), FastTrigCalculator.CosRadApprox(theta), 0);

				v.angle = FastTrigCalculator.getAngleTo(v.pos.x, v.pos.y, true);
				v.pos *= lightRadius;



				RaycastHit2D ray = Physics2D.Raycast(transform.position,v.pos,lightRadius, interactionLayers);

				//If we didn't hit anything, we need to make sure that we still cast light there
				if (!ray) {
					allVertices.Add (v);
				} 

			}

		}

		/**
		 * This returns true if there's a chance that the collider stretches from Q1 to Q4
		 */
		private bool raycastAtCollider(PolygonCollider2D collider, List<LightInteractVertex> toAddVertsTo) {
			bool oneInQ4 = false;
			bool oneInQ1 = false;
			for (int i = 0; i < collider.GetTotalPointCount(); i++) {
				LightInteractVertex vertI = new LightInteractVertex();

				// The collider.points[i] vector contains positions relative to the collider itslef
				// We transform the point like this to bring it from colliderN.transform's locality to world locality
				Vector3 worldPoint = collider.transform.TransformPoint(collider.points[i]);
				Vector3 direction = worldPoint - transform.position; //Offset from worldPoint

				float dist = Mathf.Sqrt(Mathf.Min (lightRadius*lightRadius, direction.sqrMagnitude));
				RaycastHit2D ray = Physics2D.Raycast (transform.position, direction, dist, interactionLayers);

				if (ray) { //If this hit anything at all (it's possible to miss when a collider has a point further than lightradius but has one within lightradius
					if (almostEquals(ray.point.sqrMagnitude, worldPoint.sqrMagnitude, 0.15f)) {
						//very fast way of seeing if the point in the mesh was the point hit by the raycast
						//if that's not the case, then we know this vertexNI blocks the path from light to colliderN.points[i]
						vertI.isExpectedPosition = true;
					}
					vertI.pos = transform.InverseTransformPoint (ray.point); 
				} else {
					//If we didn't hit anything at all, we have to put the vertext for the light mesh to be in the same direction
					//but to be instead exactly lightRadius units away
					vertI.pos = transform.InverseTransformPoint (direction.normalized * lightRadius + transform.position);
					vertI.isExpectedPosition = true;
				}

				vertI.angle = FastTrigCalculator.getAngleTo(vertI.pos.x, vertI.pos.y, true);
				toAddVertsTo.Add(vertI);

				if (vertI.angle < 1.57) {
					oneInQ1 = true;
				}
				if (vertI.angle > 4.71) {
					oneInQ4 = true;
				}

				Debug.DrawLine(transform.position, transform.TransformPoint(vertI.pos), Color.white);	
			}
			return oneInQ1 && oneInQ4;
		}

		/**
		 * Now, we need to consider the arrangement of our vertices
		 * They're ordered by angle, but what if things have the same or nearly the same angle?
		 * We might have two vertices that came from cast points at different locations in the same direction
		 * but that end up casting to the same surface, and this can be visually very buggy
		 * since these are sorted, they'll be right next to each other if this is the case!!!
		 */
		void fixVertOrder () {
			//it's first important to now order every single vertext from 0-2pi
			sortList(allVertices);

			const float rangeAngleComparision = 0.00001f;
			for(int i = 0; i< allVertices.Count-1; i+=1){
				LightInteractVertex first = allVertices[i];
				LightInteractVertex second = allVertices[i +1];

				if(almostEquals(first.angle, second.angle, rangeAngleComparision)){
					/**
					 * We swap two vertices in list order iff one is the end and is closer than the other 
					 * or if one is the beginning and is further than the other
					 * 
					 * TODO: understand why this is necessary
					 */
					if((second.loc == LightInteractVertex.Location.END && first.pos.sqrMagnitude > second.pos.sqrMagnitude)
						|| (first.loc == LightInteractVertex.Location.BEGINNING && first.pos.sqrMagnitude < second.pos.sqrMagnitude)){
						allVertices[i] = second;
						allVertices[i+1] = first;
					}
				}
			}
		}

		/**
		 * Now that we have all of the vertices correctly established, we need to actually render the lightmesh
		 * 
		 * Notes:
		 * 	Conceptually, a mesh is just a collection of two things:
		 * 		1) Vertices, a bunch of coordinates in the world
		 * 		2) Triangles, information that details where these vertices are connected
		 * 
		 * 	However, to actually render anything on these, we do what's called UV Mapping.
		 *  	A UV map is something that takes a 2D texture and wraps it onto some sort of distorted mesh. 
		 * 
		 * 	Therefore, to render the light mesh, we first give the mesh all of the vertices, then programatically describe how they're connected (triangles)
		 * 	and along with that, we must provide the mesh.uv coordinates for the texture to wrap to.
		 * 
		 * 	Fortunately for us, this doesn't have to be that hard, as we can simply map the texture's 2d coordinates onto the 3d position
		 * 	by iterating through every vertice and adding a uv value for the x and y coordinates of each vertice in the mesh's vertices
		 */
		private void renderLightMesh(){

			//-------------------VERTICES-----------------//
			//This array is what we will hold all the mesh's vectors in 
			Vector3[] initVerticesMeshLight = new Vector3[allVertices.Count+1];
			//Begin the mesh at the light's position
			initVerticesMeshLight[0] = Vector3.zero;
			for (int i = 0; i < allVertices.Count; i++) {
				initVerticesMeshLight [i+1] = allVertices[i].pos;
			}
			lightMesh.Clear ();
			lightMesh.vertices = initVerticesMeshLight;

			//-------------------TRIANGLES-----------------//
			int idx = 1;
			int [] triangles = new int[(allVertices.Count * 3)];
			//The way Unity handles meshes, the meshs' triangle array is 3 times the size of its vertex array
			//and groupings of three indices define each triangle. The values of each item in the array
			//must be the index into the mesh vertex array for the desired point
			for (int i = 0; i < (allVertices.Count*3); i+= 3) {
				//We iterate by i+=3 each time, so each loop defines one triangle
				//What should a triangle look like? 
				//Should connect the light casted position (the light mesh's 0th index'd coordinate)
				//to the next two points in the angularly-sorted vertex array

				triangles[i] = 0; //The light cast point

				triangles[i+1] = idx; //The next item in the vertex array
				triangles[i+2] = idx+1; //The item after that (will be also connected to with next triangle)

				//However, the very final triangle needs to connect back to the beginning of the first triangle
				if(i == (allVertices.Count*3)-3){
					triangles[i+2] = 1;	
				}

				idx++;
			}
			lightMesh.triangles = triangles;												

			//---------------------UVS-------------------//
			Vector2 [] uvs = new Vector2[initVerticesMeshLight.Length];
			for (int i = 0; i < initVerticesMeshLight.Length; i++) {
				uvs[i] = new Vector2(initVerticesMeshLight[i].x, initVerticesMeshLight[i].y);		
			}
			lightMesh.uv = uvs;


			//Update sharedMaterial in the renderer to go back to lightMaterial (in case other scripts do the same)
			GetComponent<Renderer>().sharedMaterial = lightMaterial;
		}

		/**
		 * Resets the lightmesh's bounds to be centered at (0, 0, 0)
		 * For meshes, the bound's coordinates are referenced locally
		 * That is to say that it's not affected by the parent object's 
		 * transform in the world, therefore, (0,0,0) is actually 
		 * the transform's current position.
		 */
		private void resetBounds(){
			Bounds newBound = lightMesh.bounds;
			newBound.center = Vector3.zero;
			lightMesh.bounds = newBound;
		}







		// -----------------------HELPER FUNCTIONS----------------------- //

		private bool almostEquals(float a, float b, float epsilon) {
			return a == b || Mathf.Abs (Mathf.Abs (a) - Mathf.Abs (b)) < epsilon;
		}
			
		private void setupMeshFilter() {
			// Add a Mesh Filter component to the light game object so it can take on a form,
			// and a Mesh Renderer component to the light game object so the form can become visible
			MeshFilter meshFilter = (MeshFilter)gameObject.AddComponent(typeof(MeshFilter));
			MeshRenderer renderer = (MeshRenderer)gameObject.AddComponent(typeof(MeshRenderer));	


			// For a MeshRenderer, sharedMaterial is a statically used material for all meshRendererers,
			// whereas 'material' is simply used by this instance of renderer,
			// so for performance-sensitive stuff like this, we should use sharedMaterial and avoid updating individual materials
			renderer.sharedMaterial = lightMaterial;

			//This creates the mesh, associates our meshfilter with it, names it, and optimizes it for frequent changes
			lightMesh = new Mesh();
			meshFilter.mesh = lightMesh;
			lightMesh.name = "Light Mesh";
			lightMesh.MarkDynamic ();
		}
			

		private void sortList(List<LightInteractVertex> lista){
			lista.Sort ((item1, item2) => (item1.angle.CompareTo (item2.angle)));
		}
	}
}

