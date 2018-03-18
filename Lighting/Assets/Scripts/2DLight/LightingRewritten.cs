
namespace LightingRewritten
{
	using UnityEngine;
	using System.Collections;
	using System.Collections.Generic;


	public class LightingRewritten : MonoBehaviour {
		/**
	 	* The dynamic lighting parses the meshes in the world 
		 * and stores the useful information from them as type Vertex
	 	*/
		private class LightInteractVertex
		{
			public enum Significance {NONE, LOWANGLE, HIGHANGLE};
			public Significance sig {get;set;} 

			public float angleRelToLight {get;set;}
			public float angleRelToCollider { get; set;}

			public Vector3 positionRelToLight {get;set;}
			public bool hitIntendedPolygon { get; set;}

		}
			

		[Header("Light Display Properties")]
		[Tooltip("The material overlaid on the area intended to be illuminated")]
		public Material lightMaterial;
		[Tooltip("This is the radius for how far in the game coordinates the light should raycast and extend to")]
		[Range(1, 500)]
		public float lightRadius = 20f;

		[Header("Behavior")]
		[Tooltip("The layers for the lighting to interact with")]
		public LayerMask interactionLayers;
		[Tooltip("Check this box to restrict the light being casted from this light to a specific angle")]
		public bool angledLight = false;
		[Tooltip("The angle, in degrees, that defines the width of where light will be cast")]
		[Range(1, 360)]
		public int degreeOfLightCast = 360;
		[Tooltip("The angle, in degrees, that the light is facing (lower bound)")]
		[Range(0, 359)]
		public float angleFacing = 0;

		[Header("Optimizations")]
		[Tooltip("Assume all colliders in layermask are PolygonCollider2Ds")]
		public bool onlyPolyColliders;
		[Tooltip("Use only approximate trig values each frame. Checking this box will help performance, reduce precision")]
		public bool approximateTrigVals = true;
		[Tooltip("If using approximate trig vals, this will increase performance in game, but increase load times as well")]
		public bool calculateApproxTrigValsOnLoad = true;
		[Tooltip("Increasing this will increase loading times, but also increase the accuracy of the simulation")]
		[Range(360, 3600)]
		public int loadedTrigValPrecision = 720;
		[Tooltip("The number of rays casted freely into space. Reducing this will increase performance, but make edges of light 'jaggier'")]
		[Range(8,100)]
		public int lightSegments = 8;

		[Header("Debug Lighting Visuals")]
		public bool renderLight = true;
		public bool useEqRays = true;
		public bool usePolyCollRays = true;
		public bool useEdgeCheckRays = true;
		[Header("Debug Scene Display Settings")]
		public bool highlightCorners = false;
		public bool drawDebugLines = true;
		public bool drawEqRays = false;
		public bool drawMeshVertConnections = false;
		public bool uniformColoredSortOrder = false;

	
		private PolygonCollider2D[] allColliders;
		private List<LightInteractVertex> allVertices;
		private Mesh lightMesh;
		private float angleFacingRad;

		//Necessary for rotation
		private float angleLimRad;

		void Start () {
			if (calculateApproxTrigValsOnLoad) {
				//loads lots of default vals for trig ops
				FastTrigCalculator.memoize (loadedTrigValPrecision);
			}

			allVertices = new List<LightInteractVertex>();
			angleLimRad = degreeOfLightCast * Mathf.Deg2Rad;
			MeshFilter meshFilter = (MeshFilter)gameObject.AddComponent(typeof(MeshFilter));
			MeshRenderer meshRenderer = (MeshRenderer)gameObject.AddComponent(typeof(MeshRenderer));	


			// For a MeshRenderer, sharedMaterial is a statically used material for all meshRendererers,
			// whereas 'material' is simply used by this instance of renderer,
			// so for performance-sensitive stuff like this, we use sharedMaterial and avoid updating individual materials
			meshRenderer.sharedMaterial = lightMaterial;

			//This creates the mesh, associates our meshfilter with it, and optimizes it for frequent changes
			lightMesh = new Mesh();
			meshFilter.mesh = lightMesh;
			lightMesh.MarkDynamic ();

		}

		/**
		 * Logic for calculating and updating dynamic light on every frame
		 * Five steps:
		 * 	1) Populate allMeshes with the PolygonCollider2Ds of everything in the scene within a certain
		 * 		radius of the current location of the light
		 * 	2) Create all of the vertices we'll use to assemble the mesh
		 * 	3) Sort through every vertex, and manage any conflicts that appear before rendering
		 * 	4) Use these vertices to actually generate the mesh and overlay the lightMaterial onto it
		 * 	5) Reset the bounds of the mesh so that we can update it again next frame
		 * 
		 * TODO: FixedUpdate?
		 */
		void Update(){
			//This is all necessary for rotation
			if (transform.eulerAngles.z != 0) {
				angleFacing = transform.eulerAngles.z;
				transform.eulerAngles = Vector3.zero;
			}
			angleFacingRad = angleFacing * Mathf.Deg2Rad;
			angleLimRad = degreeOfLightCast * Mathf.Deg2Rad;

			getAllMeshes();
			createVertices ();
			fixVertOrder ();
			renderLightMesh ();
			if (!renderLight) {
				lightMesh.Clear ();
			}
			resetBounds ();

		}

		void OnDrawGizmosSelected() {
			if (highlightCorners) {
				foreach (LightInteractVertex vert in allVertices) {
					if (vert.hitIntendedPolygon) {
						if (vert.sig == LightInteractVertex.Significance.HIGHANGLE) {
							Gizmos.color = Color.blue;
							Gizmos.DrawSphere (transform.TransformPoint (vert.positionRelToLight), 0.3f);
						} else if (vert.sig == LightInteractVertex.Significance.LOWANGLE) {
							Gizmos.color = Color.yellow;
							Gizmos.DrawSphere (transform.TransformPoint (vert.positionRelToLight), 0.3f);
						}

					}
				}
			}
		}


		/**
		 * Step 1:
		 * 
		 * Uses Physics2D.OverlapCircleAll to get each collider in the scene that's within
		 * the desired lighting radius and stores the polygoncollider2ds within allMeshes
		 */
		void getAllMeshes(){
			Collider2D[] colliders = Physics2D.OverlapCircleAll(transform.position, lightRadius, interactionLayers);

			//If this optimization was selected, we can just cast every collider in interactionLayers into a PolygonCollider2D to save time
			if (onlyPolyColliders) {
				allColliders = new PolygonCollider2D[colliders.Length];
				for (int i = 0; i < colliders.Length; i++) {
					allColliders [i] = (PolygonCollider2D)colliders [i];
				}

			} else {
				//Block of code below figures out which colliders are polygoncolliders
				int numPolygonColliders = 0;
				bool[] isPolyCollider = new bool[colliders.Length];
				for (int i=0; i<colliders.Length; i++) {
					if (colliders [i] is PolygonCollider2D) {
						numPolygonColliders += 1;
						isPolyCollider [i] = true;
					}
				}
				//Block of code below uses that information to copy only the polygoncolliders into allMeshes
				allColliders = new PolygonCollider2D[numPolygonColliders];
				for (int i = 0; i < numPolygonColliders; i++) {
					if (isPolyCollider [i]) {
						allColliders [i] = (PolygonCollider2D)colliders [i];
					}
				}
			}
		}

		/**
		 * Step 2:
		 * 
		 * This is the most complicated, performance-intensive step, as this is where we actually generate 
		 * all of the vertices that we use to build up the lightmesh for the scene.
		 * 
		 * To do this, two steps are performed:
		 * 		First, we raycast lightSegments different rays outwards from the light source,
		 * 		each equidistant from their neighbors in their rotation. This gives us a 
		 * 		baseline, rough picture of where the light hits, but if one wanted precise
		 * 		visuals with this method, lightSegments would need to be impossibly high,
		 * 		leading us to the need for the next step.
		 * 
		 * 		Second, we raycast at each point on each polygoncollider2d within lightRadius of the light.
		 * 		This is done in lieu of raycasting a million segments, as we can have precise knowledge of
		 * 		exactly what parts of polygons are blocked from the light and what aren't, without having 
		 * 		to send a large amount of rays towards space between points on each individual polygon,
		 * 		as we would if we only implemented step 1. However, this leads us with poor visual behavior
		 * 		when no ray is near the corner that stops blocking the light, as there's nothing calculated
		 * 		to precisely place a vertex right past each light-blocking corner. So, we locate each corner 
		 * 		that forms an endpoint on the surface visible to the light, and we raycast slightly off from 
		 * 		each corner, allowing us to find where the light will hit immediately as it passes each 
		 * 		light-blocking corner of each polygonCollider.
		 * 
		 * This leaves us with all the vertices we need to genrate the mesh properly
		 */

		void createVertices() {
			/**
			 * allVertices is populated each frame so we need to wipe it each time
			 * TODO: check to see what's quicker: 
			 * 		1) making a new list here, or 
			 * 		2) clearing the existing one and never allocating more memory
			 */
			allVertices.Clear ();

			//STEP 1
			if (useEqRays) sendEquidistentRays();

			//STEP 2
			if (usePolyCollRays) sendColliderRays();
		}


		private void sendEquidistentRays() {
			float theta = (angledLight) ? angleFacingRad : 0;
			float amount = ((angledLight) ? angleLimRad : Mathf.PI * 2) / (float)lightSegments;

			for (int i = 0; i < lightSegments; i++)  {
				LightInteractVertex vert = new LightInteractVertex();

				//Creates a position vector of magnitude lightRadius in the angle defined by this loop
				float sine = (approximateTrigVals) ? FastTrigCalculator.SinRadApprox(theta) : Mathf.Sin(theta);
				float cosine = (approximateTrigVals) ? FastTrigCalculator.CosRadApprox(theta) : Mathf.Cos(theta);
				vert.positionRelToLight = new Vector3(cosine, sine, 0);
				vert.positionRelToLight *= lightRadius;
				//Must save angle calculated by same fxn for every vert
				vert.angleRelToLight = FastTrigCalculator.getAngleTo(vert.positionRelToLight.x, vert.positionRelToLight.y, approximateTrigVals);
				//Now, send a ray from the lightsource to the position we just created
				RaycastHit2D ray = Physics2D.Raycast(transform.position,vert.positionRelToLight,lightRadius, interactionLayers);

				if (ray) {
					//it will still have the same exact angle as before but just a new position
					vert.positionRelToLight = transform.InverseTransformPoint (ray.point);
				}	
				allVertices.Add (vert);
				if (drawEqRays && !uniformColoredSortOrder) Debug.DrawLine(transform.position, transform.TransformPoint(vert.positionRelToLight), Color.magenta);	

				theta = (theta + amount) % (Mathf.PI * 2);
			}


		}

		private void sendColliderRays() {
			List <LightInteractVertex> tempVerts = new List<LightInteractVertex>();

			//We want to go through every mesh in the light radius' range and see where on them the light hits
			for (int n = 0; n < allColliders.Length; n++) {
				PolygonCollider2D colliderN = allColliders[n];

				tempVerts = raycastAtCollider (colliderN);

				if (tempVerts.Count == 0) {
					continue;
				}
				if (tempVerts.Count == 1) {
					allVertices.Add (tempVerts [0]);
					if (useEdgeCheckRays) {
						checkEdge (tempVerts [0]);
					}
				}
				//This should sort vertices from lowest to highest angles in the tempVerts list
				tempVerts.Sort ((v1, v2) => (v1.angleRelToCollider.CompareTo(v2.angleRelToCollider)));

				int posLowAngle = 0;
				int posHighAngle = tempVerts.Count - 1;

				tempVerts[posLowAngle].sig = LightInteractVertex.Significance.LOWANGLE;
				tempVerts[posHighAngle].sig = LightInteractVertex.Significance.HIGHANGLE;

				allVertices.AddRange(tempVerts);

				/**
				 * At this point, we've added the vertices and their illumination points to the list
				 * All that we need to do is add vertices that we create directly past the edge of wherever the light is touching
				 * We already have identified the edges that touch the currently polygoncollider2d and which ones are 
				 * the furthest apart, so now we just need to search around that area
				 */
				if (useEdgeCheckRays) {
					checkEdge (tempVerts [posLowAngle]);
					checkEdge (tempVerts [posHighAngle]);
				}

			}
		}

		private List<LightInteractVertex> raycastAtCollider(PolygonCollider2D collider) {
			List<LightInteractVertex> thisCollider = new List<LightInteractVertex> ();

			for (int i = 0; i < collider.GetTotalPointCount (); i++) {
				LightInteractVertex vert = new LightInteractVertex();
				// The collider.points[i] vector contains positions relative to the collider itslef
				// We transform the point like this to bring it from colliderN.transform's locality to world locality
				Vector3 worldPoint = collider.transform.TransformPoint(collider.points[i]);
				Vector3 direction = worldPoint - transform.position; //Offset from worldPoint
				vert.angleRelToLight = FastTrigCalculator.getAngleTo(direction.x, direction.y, approximateTrigVals);

				//If we're using angled light and this point isn't in the angle, don't bother doing any of this
				if (angledLight && !withinRange (vert.angleRelToLight, angleFacingRad, angleFacingRad + angleLimRad)) {
					continue;
				}

				//TODO: consider doing this instead: float dist = Mathf.Sqrt(Mathf.Min (lightRadius*lightRadius, direction.sqrMagnitude));
				float dist = direction.magnitude;
				RaycastHit2D ray = Physics2D.Raycast (transform.position, direction, dist, interactionLayers);

				if (ray) { //If this hit anything at all (it's possible to miss when a collider has a point further than lightradius but has one within lightradius
					if (transform.InverseTransformPoint (ray.point).sqrMagnitude > lightRadius * lightRadius) {
						vert.positionRelToLight = direction.normalized * lightRadius;
					} else {
						if (almostEquals(ray.point.sqrMagnitude, worldPoint.sqrMagnitude, 0.15f)) {
							//very fast way of seeing if the point in the mesh was the point hit by the raycast
							//if that's not the case, then we know this vertexNI blocks the path from light to colliderN.points[i]
							vert.hitIntendedPolygon = true;
						}
						vert.positionRelToLight = transform.InverseTransformPoint (ray.point); 
					}
				} else {
					//If we hit nothing, this is only because of floating point rounding error causing us to miss
					//So we can simply assume that we did actually hit the corner
					vert.positionRelToLight = (direction.sqrMagnitude > lightRadius * lightRadius) ? direction.normalized * lightRadius : direction; 
					vert.hitIntendedPolygon = true;
				}
				thisCollider.Add(vert);
				//We need the angle rel to collider for sorting, but vec3.angle gives only positive angles
				vert.angleRelToCollider = Vector3.Angle (thisCollider [0].positionRelToLight, vert.positionRelToLight);
				//Solution: use cross product. 
				vert.angleRelToCollider = (Vector3.Cross(thisCollider [0].positionRelToLight, vert.positionRelToLight).z < 0) ? vert.angleRelToCollider : -vert.angleRelToCollider;
				if (drawDebugLines && !uniformColoredSortOrder) Debug.DrawLine(transform.position, transform.TransformPoint(vert.positionRelToLight), Color.white);
			}
			return thisCollider;
		}


		/**
		 * This function is sort of confusing, so to document:
		 * 
		 * 	We take the position (world coordinates) of the vertex on the edge of the polygon
		 * 	We take the direction from the light to the polygon
		 * 	Then, we fudge the location of that vertex by a bit in that same direction,
		 * 		and since that direction is the direction that got us there, if this is a corner,
		 * 		we will slip just slightly past the corner and see what is past it.
		 * 	We then raycast from that fudged point in the same direction by the amount
		 * 	that we can reach with our lightradius defined minus how far the vertex was
		 * 
		 * 	After that, we either hit something, and we can add that point, or we didn't,
		 * 	and we need to add the point at lightradius away from light in same direction
		 */
		private void checkEdge(LightInteractVertex vert) {
			if(vert.hitIntendedPolygon){ //If it was obstructed at the very edge angles, we don't need to bother
				Vector2 from = transform.TransformPoint (vert.positionRelToLight); 
				Vector2 dir = (Vector2) vert.positionRelToLight; 
				Vector2 off = dir * 0.01f;
				from += off;
				float dist = lightRadius - dir.magnitude;
				RaycastHit2D peekRay = Physics2D.Raycast(from, dir, dist, interactionLayers);

				Vector3 worldPointNewVert;

				LightInteractVertex vL = new LightInteractVertex();

				if(peekRay){
					//This means that fudging the edge very slightly made us hit another object which we want to shine light on
					worldPointNewVert = peekRay.point;
				}else{
					worldPointNewVert = from + dir.normalized * dist;
				}

				if (drawDebugLines && !uniformColoredSortOrder) Debug.DrawLine(from - off, worldPointNewVert, Color.green);


				vL.positionRelToLight = transform.InverseTransformPoint (worldPointNewVert);
				vL.angleRelToLight = FastTrigCalculator.getAngleTo(vL.positionRelToLight.x, vL.positionRelToLight.y, approximateTrigVals);
				allVertices.Add(vL);
			}
		}

		/**
		 * Now, we need to consider the arrangement of our vertices
		 * They're ordered by angle, but what if things have the same or nearly the same angle?
		 * We might have two vertices that came from cast points at different locations in the same direction
		 * but that end up casting to the same surface, and this can be visually very buggy
		 * since these are sorted, they'll be right next to each other if this is the case!!!
		 */
		private void fixVertOrder () {
			//it's first important to now order every single vertext from 0-2pi
			allVertices.Sort ((v1, v2) => (v1.angleRelToLight.CompareTo(v2.angleRelToLight)));
			if (drawDebugLines && uniformColoredSortOrder) orderedColoredDebug ();

			const float rangeAngleComparision = 0.0001f;
			for(int i = 0; i< allVertices.Count-1; i+=1){
				LightInteractVertex first = allVertices[i];
				LightInteractVertex second = allVertices[i +1];



				if(almostEquals(first.angleRelToLight, second.angleRelToLight, rangeAngleComparision)){
					//This needs to happen because of our implementation of checkEdge
					if (first.sig == LightInteractVertex.Significance.HIGHANGLE && second.positionRelToLight.sqrMagnitude > first.positionRelToLight.sqrMagnitude) {
						allVertices [i] = second;
						allVertices [i + 1] = first;
					}

					if (second.sig == LightInteractVertex.Significance.LOWANGLE && first.positionRelToLight.sqrMagnitude > second.positionRelToLight.sqrMagnitude) {
						allVertices [i] = second;
						allVertices [i + 1] = first;
					}
				}
			}
		}


		private void orderedColoredDebug() {
			Color c = new Color ();
			c.a = 1f;
			c.b = 1f;
			c.r = 0.5f;
			c.g = 0f;
			float incAm = 1f / allVertices.Count;
			for (int i = 0; i < allVertices.Count; i++) {
				Debug.DrawRay (transform.position, allVertices [i].positionRelToLight, c);
				c.b -= incAm;
				c.r += incAm / 2;
				c.g += incAm;
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
			lightMesh.Clear ();
			GetComponent<Renderer>().sharedMaterial = lightMaterial;
			lightMesh.vertices = initVertices();
			lightMesh.triangles = initTriangles();												
			lightMesh.uv = initUVs();
		}

		private Vector3[] initVertices() {
			Vector3[] initVerticesMeshLight = new Vector3[allVertices.Count+1];
			//Begin the mesh at the light's position
			initVerticesMeshLight[0] = Vector3.zero;
			for (int i = 0; i < allVertices.Count; i++) {
				initVerticesMeshLight [i+1] = allVertices[i].positionRelToLight;
			}
			return initVerticesMeshLight;
		}

		private Vector2[] initUVs() {
			Vector2 [] uvs = new Vector2[lightMesh.vertices.Length];
			for (int i = 0; i < lightMesh.vertices.Length; i++) {
				uvs[i] = new Vector2(lightMesh.vertices[i].x, lightMesh.vertices[i].y);		
			}
			return uvs;
		}

		private int[] initTriangles() {
			//The way Unity handles meshes, the mesh's triangle array is 3 times the size of its vertex array
			//and groupings of three indices define each triangle. The values of each item in the array
			//must be the index into the mesh vertex array for the desired point

			int [] triangles = new int[(allVertices.Count * 3)];
			for (int i = 0; i < allVertices.Count; i++) {
				//If we're angling the light, we don't want to connect the last angles
				if (angledLight && i == allVertices.Count - 1) {
					break;
				}
				//We iterate by i+=3 each time, so each loop defines one triangle
				//What should a triangle look like? 
				//Should connect the light casted position (the light mesh's 0th index'd coordinate)
				//to the next two points in the angularly-sorted vertex array

				triangles[i*3] = 0; //The light cast point

				triangles[i*3 + 1] = i+1; 
				triangles[i*3 + 2] = i+2; 

				//However, the very final triangle needs to connect back to the beginning of the first triangle
				if(i == allVertices.Count - 1){
					triangles[i*3 + 2] = 1;	
				}
				if (drawMeshVertConnections) Debug.DrawLine(transform.TransformPoint(allVertices[triangles[i*3 + 1]-1].positionRelToLight), transform.TransformPoint(allVertices[triangles[i*3 + 2]-1].positionRelToLight), Color.white);
			}
			return triangles;
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

		private bool withinRange(float value, float lower, float upper) {
			return value > lower && value < upper;
		}


	}
}

