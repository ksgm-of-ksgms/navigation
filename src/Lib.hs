{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE FlexibleInstances, MultiParamTypeClasses, FunctionalDependencies #-}


module Lib
    ( someFunc
    ) where


import Data.Maybe ( fromMaybe )
import Data.Int (Int32)
import Control.Applicative
import Linear.V3
import Linear.Matrix
import Graphics.UI.GLUT ( Key(..), KeyState(..) )
import Vis
import Vis.Vis ( vis )
import Vis.Camera ( makeCamera, setCamera, cameraMotion )
import Control.Lens

rotMat33X :: (Floating f) => f -> M33 f
rotMat33X a = V3 (V3 1 0 0) (V3 0 ca (-sa))  (V3 0 sa ca) where
    ca = cos a
    sa = sin a

rotMat33Z :: (Floating f) => f -> M33 f
rotMat33Z a = V3 (V3 ca (-sa) 0)  (V3 sa ca 0) (V3 0 0 1) where
    ca = cos a
    sa = sin a

--rotation = _m33

rotMat44X :: (Floating f) => f -> M44 f
rotMat44X a = identity & _m33 .~ rotMat33X a

rotMat44Z :: (Floating f) => f -> M44 f
rotMat44Z a = identity & _m33 .~ rotMat33Z a

transMat44X :: (Floating f) => f -> M44 f
transMat44X a =  identity & translation .~ V3 a 0 0

transMat44Z :: (Floating f) => f -> M44 f
transMat44Z a =  identity & translation .~ V3 0 0 a

---- i-th Link
--class IOpenLinkElm elm where
--    Rt :: (Floating f) => elm -> f -> (M33 f, V3 f) -- calc frame{i+1} with θ_{i+1}
--    m4 :: (Floating f) => elm -> f -> M44

class Renderable b f  | b -> f where
    render :: b -> VisObject f

-- M44 を render
instance (Floating f) => Renderable (M44 f) f where
    render m = Trans t $ RotDcm rot $ Axes (0.5, 1)
        where
        rot = m ^. _m33
        t   = m ^. translation

data DH a = DH {   __a :: a      -- [m]
               ,   __d :: a      -- [m]
               ,   __alpha :: a  -- [rad]
               } deriving Show

makeLenses ''DH

---- for rendering and collision detection
--data LinkShape a = AABB (a, a) (a, a) (a, a)
--
d1 = 0.089159
a2 = 0.425
a3 = 0.39225
d4 = 0.10915
d5 = 0.09465
d6 = 0.0823

ur5dh :: [DH Double]
ur5dh =
    [ DH 0             0           0      -- base
    , DH 0             d1          (pi/2)
    , DH (-a2)         0           0
    , DH (-a3)         0           0
    , DH 0             d4          (pi/2)
    , DH 0             d5          (-pi/2)
    , DH 0             d6          0      -- flange
    ]

---- how to draw link (from frame{i} to frame{i+1})
----ur5box :: (Floating f) => [LinkShape f]
----ur5box =
----    [ AABB (-r, r) (-r, r) (0, d1)
----    , AABB (-a2-r, r) (-r, r) (0, 2*r)
----    , AABB (-a3-r, r) (-r, r) (-2*r, 0)
----    -- , AABB 
----    -- , AABB 
----    -- , AABB 
----    -- , AABB 
----    ]
--
newtype FrameTrans a = FrameTrans { _matT :: a -> M44 a }

makeLenses ''FrameTrans

-- T : frame {i} in frame {i-1} = Rx(α[i-1]) Dx(a[i-1]) Rz(θ[i]) Dz(d[i])
dr2ft :: (Floating f) => DH f -> DH f-> FrameTrans f
dr2ft dh0 dh1 = FrameTrans f
    where
    f theta = rx !*! dx !*! rotMat44Z theta !*! dz
    rx = rotMat44X $ dh0 ^. _alpha
    dx = transMat44X $ dh0 ^. _a
    dz = transMat44Z $ dh1 ^. _d

drs2fts :: (Floating f) => [DH f] -> [FrameTrans f]
drs2fts [dh0,dh1]     = [dr2ft dh0 dh1]
drs2fts (dh0:dh1:dhs) = dr2ft dh0 dh1 : drs2fts (dh1:dhs)
drs2fts _ = error "invalid args"


ur5fts :: [FrameTrans Double]
ur5fts = drs2fts ur5dh

-- calc each frame in base-coordinate for each joint-angle
kinematics :: (Floating f) => [FrameTrans f] -> [f] -> [M44 f]
kinematics (e:es) (a:as) = t : ((t !*! ) <$> kinematics es as)
    where
    t = (e ^. matT) a
kinematics [] [] = []
kinematics _ _ = error "invalid args"

newtype World = World Double

mySimulateIO :: IO ()
mySimulateIO = vis defaultOpts ts (world0, cameraState0) simFun drawFun setCameraFun (Just kmCallback) (Just motionCallback) Nothing
  where

    ts = 0.01
    world0 = World 0.1
    defaultCamera = Camera0 { phi0 = 60 , theta0 = 20 , rho0 = 7 }
    cameraState0 = makeCamera $ fromMaybe defaultCamera (optInitialCamera defaultOpts)

    simFun ((world,cameraState),time) = 
      return (world, cameraState)

    drawFun ((World n, _),_) = do
      let obs = render (identity :: M44 Double)
      --Cube n Solid blue
      return (obs, Nothing)

    setCameraFun (_,cameraState) = setCamera cameraState

    kmCallback (world, camState) k0 k1 _ _ = (world', camState) where
        world' = case (k0,  k1) of 
                         (Char 'j', Down) -> World 0.2
                         (Char 'k', Down) -> World 0
                         (_, _)           -> world

    motionCallback (world, cameraState) pos = (world, cameraMotion cameraState pos)


someFunc :: IO ()
someFunc = do
    --print $ kinematics ur5fts [0, 0, 0, 0, 0, 0]
    mySimulateIO
    return ()

