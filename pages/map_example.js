import Link from 'next/link'
import InteractiveMap from 'components/interactive-map.js'

import {Container, Row, Col} from 'reactstrap'

export default () => (
  <Container>
    <Row>
      <Col>
        <InteractiveMap />
      </Col>
    </Row>
  </Container>
)
