#! /usr/bin/sed -f
s/b2//g
s/->/./g
s/Body\*/Body/g
s/Joint\*/Joint/g
s/Shape\*/Shape/g
s/Fixture\*/Fixture/g
s/(Vec2(/(new Vec2(/g
s/, Vec2(/, new Vec2(/g
s/\.Set/.set/g
s/\.Get/.get/g
s/\.Cre/.cre/g
s/\.Ini/.ini/g
s/\.Des/.des/g
s/NULL/null/g
s/const /final /g
s/float32/float/g
s/bool /boolean /g
s/int32/int/g
s/&bd/bd/g
s/&fd/fd/g
s/&shape/shape/g
s/BodyDef bd;/BodyDef bd = new BodyDef();/g
s/PolygonShape shape;/PolygonShape shape = new PolygonShape();/g
s/CircleShape shape;/CircleShape shape = new CircleShape();/g
s/FixtureDef fd;/FixtureDef fd = new FixtureDef();/g
s/m_world/world/g
s/_pi/MathUtils.PI/g
s/RandomFloat/MathUtils.randomFloat/g
s/_dynamicBody/BodyType.DYNAMIC/g
s/Abs(/MathUtils.abs(/g
s/ sin(/ MathUtils.sin(/g
s/ sinf(/ MathUtils.sin(/g
s/ cos(/ MathUtils.cos(/g
s/ cosf(/ MathUtils.cos(/g
s/m_debugDraw/debugDraw/g
s/Color(/new Color3f(/g
s/\.Draw/.draw/g
s/Cross(/Vec2.cross(/g
s/Dot(/Vec2.dot(/g
s/\.Length/.length/g
s/_linearSlop/Settings.linearSlop/g
s/Assert(/assert(/g
s/Max(/MathUtils.max(/g
s/Min(/MathUtils.min(/g