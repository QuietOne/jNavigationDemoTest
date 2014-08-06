package navigation.test.control;

import com.jme3.ai.agents.util.control.Game;
import com.jme3.ai.navigation.detour.DetourBuilder;
import com.jme3.ai.navigation.detour.NavMesh;
import com.jme3.ai.navigation.detour.NavMeshCreateParams;
import com.jme3.ai.navigation.detour.NavMeshQuery;
import com.jme3.ai.navigation.detour.Status;
import com.jme3.ai.navigation.recast.CompactHeightfield;
import com.jme3.ai.navigation.recast.Config;
import com.jme3.ai.navigation.recast.Context;
import com.jme3.ai.navigation.recast.ContourSet;
import com.jme3.ai.navigation.recast.Heightfield;
import com.jme3.ai.navigation.recast.PolyMesh;
import com.jme3.ai.navigation.recast.PolyMeshDetail;
import com.jme3.ai.navigation.recast.RecastBuilder;
import com.jme3.ai.navigation.tilecache.TileFlags;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;

/**
 *
 * @author Tihomir Radosavljevic
 * @version 1.0
 */
public class NavGeneration {

    private Node scene;
    private Config config;
    private Material material;
    
    public NavGeneration() {
        this.scene = (Node) Game.getInstance().getRootNode().getChild(0);
        material = new Material(Game.getInstance().getApp().getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
    }

    public void makeNavMesh() {
        // Step 1. Initialize build config.
        config = new Config();

        // I have three meshes so I have to gather data from all of them and to merge
        // in one (there is probably another way to do this, but I don't know how yet)
        Vector3f[] minBoundsVector3fs = new Vector3f[3];
        Vector3f[] maxBoundsVector3fs = new Vector3f[3];
        for (int i = 0; i < 3; i++) {
            Geometry geom = (Geometry) ((Node) scene.getChild(i)).getChild(0);
            minBoundsVector3fs[i] = RecastBuilder.calculateMinBounds(geom.getMesh());
            maxBoundsVector3fs[i] = RecastBuilder.calculateMaxBounds(geom.getMesh());
        }

        Vector3f minBounds = RecastBuilder.calculateMinBounds(minBoundsVector3fs);
        Vector3f maxBounds = RecastBuilder.calculateMaxBounds(maxBoundsVector3fs);

        config.setMaxBounds(maxBounds);
        config.setMinBounds(minBounds);

        config.setCellSize(0.3f);
        config.setCellHeight(0.2f);
        config.setWalkableSlopeAngle(45);
        config.setWalkableClimb(1);
        config.setWalkableHeight(2);
        config.setWalkableRadius(2);
        config.setMinRegionArea(8);
        config.setMergeRegionArea(20);
        config.setBorderSize(20);
        config.setMaxEdgeLength(12);
        config.setMaxVerticesPerPoly(6);
        config.setDetailSampleMaxError(1f);
        config.setDetailSampleDistance(6);

        RecastBuilder.calculateGridWidth(config);
        RecastBuilder.calculatesGridHeight(config);

        // Step 2. Rasterize input polygon soup.

        //context is needed for logging that is not yet fully supported in native library.
        //It must NOT be null.
        Context context = new Context();

        // Allocate voxel heightfield where we rasterize our input data to.
        Heightfield heightfield = new Heightfield();

        if (!RecastBuilder.createHeightfield(context, heightfield, config)) {
            System.out.println("Could not create solid heightfield");
            return;
        }

        // Allocate array that can hold triangle area types. 

        // In Recast terminology, triangles are what indices in jME is. I left this,
        // because if something about recast logic, is not working, you can always ask in 
        // recast group https://groups.google.com/forum/#!forum/recastnavigation

        // Find triangles which are walkable based on their slope and rasterize them.
        // If your input data is multiple meshes, you can transform them here, calculate
        // the area type for each of the meshes and rasterize them.
        for (int i = 0; i < 3; i++) {
            Geometry geom = (Geometry) ((Node) scene.getChild(i)).getChild(0);
            char[] areas = RecastBuilder.markWalkableTriangles(context, config.getWalkableSlopeAngle(), geom.getMesh());
            RecastBuilder.rasterizeTriangles(context, geom.getMesh(), areas, heightfield, 20);
        }
        
        // Step 3. Filter walkables surfaces.
        // Once all geometry is rasterized, we do initial pass of filtering to
        // remove unwanted overhangs caused by the conservative rasterization
        // as well as filter spans where the character cannot possibly stand.
        RecastBuilder.filterLowHangingWalkableObstacles(context, config.getWalkableClimb(), heightfield);
        RecastBuilder.filterLedgeSpans(context, config, heightfield);
        RecastBuilder.filterWalkableLowHeightSpans(context, config.getWalkableHeight(), heightfield);
        
        
        // Step 4. Partition walkable surface to simple regions.
        // Compact the heightfield so that it is faster to handle from now on.
        // This will result more cache coherent data as well as the neighbours
        // between walkable cells will be calculated.
        CompactHeightfield compactHeightfield = new CompactHeightfield();
        
        if (!RecastBuilder.buildCompactHeightfield(context, config, heightfield, compactHeightfield)) {
            System.out.println("Could not build compact data");
            return;
        }

        if (!RecastBuilder.erodeWalkableArea(context, config.getWalkableRadius(), compactHeightfield)) {
            System.out.println("Could not erode");
            return;
        }

        // Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
        // There are 3 martitioning methods, each with some pros and cons:
        // 1) Watershed partitioning
        //   - the classic Recast partitioning
        //   - creates the nicest tessellation
        //   - usually slowest
        //   - partitions the heightfield into nice regions without holes or overlaps
        //   - the are some corner cases where this method creates produces holes and overlaps
        //      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
        //      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
        //   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
        // 2) Monotone partioning
        //   - fastest
        //   - partitions the heightfield into regions without holes and overlaps (guaranteed)
        //   - creates long thin polygons, which sometimes causes paths with detours
        //   * use this if you want fast navmesh generation
        String partitionType = "Sample partition watershed";

        if (partitionType.equals("Sample partition watershed")) {
            if (!RecastBuilder.buildDistanceField(context, compactHeightfield)) {
                System.out.println("Could not build distance field");
                return;
            }
            if (!RecastBuilder.buildRegions(context, compactHeightfield, config)) {
                System.out.println("Could not build watershed regions");
                return;
            }
        }

        if (partitionType.equals("Sample partition monotone")) {
            if (!RecastBuilder.buildRegionsMonotone(context, compactHeightfield, config)) {
                System.out.println("Could not build monotone regions");
                return;
            }
        }

        // Step 5. Trace and simplify region contours.
        // Create contours.
        ContourSet contourSet = new ContourSet();

        if (!RecastBuilder.buildContours(context, compactHeightfield, 2f, config.getMaxEdgeLength(), contourSet)) {
            System.out.println("Could not create contours");
            return;
        }

        // Step 6. Build polygons mesh from contours.
        // Build polygon navmesh from the contours.
        PolyMesh polyMesh = new PolyMesh();

        if (!RecastBuilder.buildPolyMesh(context, contourSet, config.getMaxVertsPerPoly(), polyMesh)) {
            System.out.println("Could not triangulate contours");
            return;
        }

        // Step 7. Create detail mesh which allows to access approximate height on each polygon.
        PolyMeshDetail polyMeshDetail = new PolyMeshDetail();

        if (!RecastBuilder.buildPolyMeshDetail(context, polyMesh, compactHeightfield, config, polyMeshDetail)) {
            System.out.println("Could not build detail mesh.");
            return;
        }

        // (Optional) Step 8. Create Detour data from Recast poly mesh.
        // The GUI may allow more max points per polygon than Detour can handle.
	// Only build the detour navmesh if we do not exceed the limit.
        if (config.getMaxVertsPerPoly() > DetourBuilder.VERTS_PER_POLYGON()) {
            return;
        }
        NavMeshCreateParams createParams = new NavMeshCreateParams();
        createParams.getData(polyMesh);
        createParams.getData(polyMeshDetail);
        //setting optional off-mesh connections (in my example there are none)
        createParams.getData(config);
        createParams.setBuildBvTree(true);
        
        char[] navData = DetourBuilder.createNavMeshData(createParams);
        
        if (navData == null) {
            System.out.println("Could not build Detour navmesh.");
            return;
        }
        
        NavMesh navMesh = new NavMesh();
        
        if (!navMesh.isAllocationSuccessful()) {
            System.out.println("Could not create Detour navmesh");
            return;
        }
        
        Status status;
        status = navMesh.init(navData, TileFlags.DT_TILE_FREE_DATA.swigValue());
        if (status.isFailed()) {
            System.out.println("Could not init Detour navmesh");
            return;
        }
        NavMeshQuery query = new NavMeshQuery();
        status = query.init(navMesh, 2048);
        
    }

    
    
    private void showMeHeightfieldSolid(Heightfield heightfield){
        Material mat = material.clone();
        mat.setColor("Color", ColorRGBA.Yellow);
        mat.getAdditionalRenderState().setWireframe(true);
        Geometry debbug = new Geometry(null);
         //= new Geometry("heightfield", heightfield.drawHeightfieldSolid());
        debbug.setMaterial(mat);
        scene.attachChild(debbug);
    }
}