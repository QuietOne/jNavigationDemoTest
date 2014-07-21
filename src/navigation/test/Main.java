package navigation.test;

import com.jme3.ai.agents.util.control.Game;
import com.jme3.ai.navigation.utils.GraphicHelper;
import com.jme3.app.SimpleApplication;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import navigation.test.control.NavGameControl;
import navigation.test.control.NavGeneration;

/**
 * test
 *
 * @author normenhansen
 */
public class Main extends SimpleApplication {

    Game game = Game.getInstance();
    
    public static void main(String[] args) {
        Main app = new Main();
        app.start();
    }

    @Override
    public void simpleInitApp() {
        //defining app
        game.setApp(this);

        //setting game control
        game.setGameControl(new NavGameControl());

        //setting camera
        game.getGameControl().setCameraSettings(cam);

        //setting flying camera
        game.getGameControl().setFlyCameraSettings(flyCam);

        //loading scene used in game from Blender file...
        ((NavGameControl) game.getGameControl()).loadScene();
        //test();

        NavGeneration ng = new NavGeneration();
        ng.makeNavMesh();
        
        //starting game
        game.start();
        stateManager.attach(game);
    }

    @Override
    public void simpleUpdate(float tpf) {
        //System.out.println(cam.getLocation());
    }
    
    private void test(){
        Mesh b = GraphicHelper.lineBox(new Vector3f(-1, -1, -1), new Vector3f(1, 1, 1));
        Mesh box = GraphicHelper.lineBox(new Vector3f(-5, -5, -5), new Vector3f(5, 5, 5));
        box.extractVertexData(b);
        box.setMode(Mesh.Mode.Lines);
        Geometry geometry = new Geometry("box", box);
        Node node = new Node("node");
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Blue);
        geometry.setMaterial(mat);
        node.attachChild(geometry);
        rootNode.attachChild(node);
    }
}
