package Solitaire.AI;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.io.File;
import java.io.IOException;
import java.util.Map;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Point;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JLayeredPane;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.WindowConstants;

public class GameGUI extends JFrame {

        private JMenuBar menuBar;

        Map<String, String> text;
        JPanel panelGameArea;
        JPanel panelColumns;
        JPanel topColumns;
        JLayeredPane lp;
        GameEngine game;
        Point mouseOffset;

        public GameGUI(GameEngine game) {
            this.game = game;
            setTitle("Solitaire");
            setSize(900, 700);
            try {
                setContentPane((new JPanelWithBackground("src/Solitaire/AI/images/background.jpg")));
            } catch (IOException e) {
                e.printStackTrace();
            }

            setLayout(new BorderLayout());

            panelGameArea = new JPanel();
            panelGameArea.setOpaque(false);
            panelGameArea.setLayout(new BoxLayout(panelGameArea, BoxLayout.PAGE_AXIS));

            setLocationRelativeTo(null);
            setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
            FlowLayout flowLayout = new FlowLayout(FlowLayout.CENTER);
            flowLayout.setAlignOnBaseline(true);

            panelColumns = new JPanel();
            panelColumns.setOpaque(false);
            panelColumns.setLayout(flowLayout);
            panelColumns.setMinimumSize(new Dimension(200, 900));

            FlowLayout topFlow = new FlowLayout(FlowLayout.LEFT);
            topFlow.setAlignOnBaseline(true);

            topColumns = new JPanel();
            topColumns.setOpaque(false);
            topColumns.setLayout(topFlow);
            panelGameArea.add(topColumns);
            panelGameArea.add(panelColumns);
            add(panelGameArea);

            lp = getLayeredPane();
            setVisible(true);

            mouseOffset = new Point(0, 0);
            initializeComponents();

            validate();
        }

        private void initializeComponents() {
            topColumns.removeAll();
            panelColumns.removeAll();

            game.setupGame();
            for (Pile pile : game.tablePiles) {
                panelColumns.add(pile);
            }

            topColumns.add(game.deckPile);
            topColumns.add(game.getPile);

            for(int i = 0; i < 4; i++) {
                Pile pile = game.foundationPiles.get(i);
                topColumns.add(pile);
            }

        }

        public void resetGame() {
            game.generateCards();
            initializeComponents();
            repaint();
        }

        public class JPanelWithBackground extends JPanel {

            private Image backgroundImage;

            public JPanelWithBackground(String fileName) throws IOException {
                File fileImage = new File(fileName);
                backgroundImage = ImageIO.read(fileImage);
            }

            public void paintComponent(Graphics g) {
                super.paintComponent(g);
                g.drawImage(backgroundImage, 0, 0, this);
            }
        }
    }

